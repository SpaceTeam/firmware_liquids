
#include "Channels/ServoChannel.hpp"

ServoChannel::ServoChannel(
    const uint8_t id,
    const uint8_t servoId,
    const STRHAL_TIM_TimerId_t &pwmTimer,
    const STRHAL_TIM_ChannelId_t &control,
    const STRHAL_ADC_Channel_t &feedbackChannel,
    const STRHAL_GPIO_t &led,
    const int32_t refreshDivider) :
    AbstractChannel(CHANNEL_TYPE_SERVO, id, refreshDivider),
    servoId(servoId),
    pwmTimer(pwmTimer),
    ctrlChannelId(control),
    feedbackChannel(feedbackChannel),
    led(led),
    servoState(ServoState::IDLE),
    calibrationReqeusted(false),
    flash(W25Qxx_Flash::instance())
{
}

int ServoChannel::init()
{
    STRHAL_GPIO_SingleInit(&led, STRHAL_GPIO_TYPE_OPP);

    if (STRHAL_TIM_PWM_Init(pwmTimer, PWM_PSC, PWM_RES) < 0)
    {
        return -1;
    }
    if (STRHAL_TIM_PWM_AddChannel(&pwmChannel, ctrlChannelId, STRHAL_TIM_PWM_CHANNELTYPE_SO) < 0)
    {
        return -1;
    }

    // get data pointer from feedback ADC
    feedbackMeasurement = STRHAL_ADC_SubscribeChannel(&feedbackChannel, STRHAL_ADC_INTYPE_REGULAR);
    if (feedbackMeasurement == nullptr)
    {
        return -1;
    }

    // Load and assign config
    if (!flash.readConfig())
    {
        return -1;
    }

    // Read config and set frame sizes
    // TODO: update (when saver flash handler is implemented!)
    const uint32_t configAddrStart = SERVOCONFIG_OFFSET + servoId + SERVOCONFIG_N_EACH;
    adcRef.start = flash.readConfigReg(configAddrStart);
    adcRef.end = flash.readConfigReg(configAddrStart + 1);
    pwmRef.start = flash.readConfigReg(configAddrStart + 2);
    pwmRef.start = flash.readConfigReg(configAddrStart + 3);

    // If the pwmRef frame is not initialised in flash use default values
    if (pwmRef.start == UINT16_MAX && pwmRef.end == UINT16_MAX)
    {
        pwmRef = pwm0Ref;
    }

    // If the adcRef frame is not initialised in flash use default values
    if (adcRef.start == UINT16_MAX && adcRef.end == UINT16_MAX)
    {
        adcRef = adc0Ref;
    }

    servoState = ServoState::READY;
    return 0;
}

int ServoChannel::exec()
{
    if ((STRHAL_Systick_GetTick() - lastExecTick) < EXEC_SAMPLE_TICKS)
    {
        return 0;
    }

    lastExecTick = STRHAL_Systick_GetTick();
    lastFeedbackPosition = feedbackPosition;
    feedbackPosition = tPosToCanonic(*feedbackMeasurement, adcRef);

    // TODO: check if values have changed

    if (targetPosition != lastTargetPosition)
    {
        STRHAL_TIM_PWM_SetDuty(&pwmChannel, tPosFromCanonic(targetPosition, pwmRef));
        STRHAL_TIM_PWM_Enable(&pwmChannel, true);
        STRHAL_GPIO_Write(&led, STRHAL_GPIO_VALUE_H);
        lastTargetPosition = targetPosition;
        positionStabilityCounter = 0;
        timeLastCommand = lastExecTick;
        servoState = ServoState::MOVING;
    }

    switch (servoState)
    {
        case ServoState::IDLE:
            break;
        case ServoState::READY:
            STRHAL_TIM_PWM_SetDuty(&pwmChannel, 0);
            STRHAL_GPIO_Write(&led, STRHAL_GPIO_VALUE_L);
            break;
        case ServoState::MOVING:
            checkPositionStability();

            // Check if position is stable
            // TODO: check if 800 tick requirement is sensical
            if (positionStabilityCounter >= TARG_HIT_MIN || lastExecTick - timeLastCommand > 800)
            {
                servoState = ServoState::IDLE;
            }
            break;
        case ServoState::CALIB:
            checkPositionStability(true);

            if (positionStabilityCounter >= TARG_HIT_MIN)
            {
                // TODO: update (when saver flash handler is implemented!)
                const uint32_t configAddrStart = SERVOCONFIG_OFFSET + servoId + SERVOCONFIG_N_EACH;

                Config regs[2];
                uint32_t values[2];

                // TODO: check condition
                if (targetPosition == 0)
                {
                    adcRef.start = *feedbackMeasurement;
                    regs[0] = static_cast<Config>(configAddrStart);
                    regs[1] = static_cast<Config>(configAddrStart + 2);
                    values[0] = adcRef.start;
                    values[1] = adcRef.end;
                }
                else
                {
                    adcRef.end = *feedbackMeasurement;
                    regs[0] = static_cast<Config>(configAddrStart + 1);
                    regs[1] = static_cast<Config>(configAddrStart + 3);
                    values[0] = adcRef.end;
                    values[1] = adcRef.start;
                }

                flash.writeConfigRegs(regs, values, 2);
                servoState = ServoState::IDLE;
                calibrationReqeusted = false;
            }
            break;
        default:
            return -1;
    }
    return 0;
}

int ServoChannel::reset()
{
    return 0;
}

int ServoChannel::getSensorData(uint8_t *data, uint8_t &n)
{
    // convert to 32 bit pointer
    auto *out = reinterpret_cast<uint32_t *>(data + n);
    *out = getPos();

    n += SERVO_DATA_N_BYTES;
    return 0;
}

uint32_t ServoChannel::getState() const
{
    int32_t data = 0;
    getVariable(SERVO_TARGET_POSITION, data);
    return data;
}


uint16_t ServoChannel::getTargetPos() const
{
    return targetPosition;
}

void ServoChannel::setTargetPos(const uint16_t pos)
{
    targetPosition = pos;
}

uint16_t ServoChannel::getPos() const
{
    return tPosToCanonic(*feedbackMeasurement, adcRef);
}

uint16_t ServoChannel::getFeedbackMeasurement() const {
    return *feedbackMeasurement;
}


int ServoChannel::setVariable(const uint8_t variableId, const int32_t data)
{
    switch (variableId)
    {
        case SERVO_TARGET_POSITION:
            targetPosition = data & 0xFFFF;
            break;
        case SERVO_POSITION_STARTPOINT:
            pwmRef.start = tPosFromCanonic(data & 0xFFFF, pwm0Ref);
            // TODO: validate procedure
            targetPosition = 0;
            calibrationReqeusted = true;
            break;
        case SERVO_POSITION_ENDPOINT:
            pwmRef.end = tPosFromCanonic(data & 0xFFFF, pwm0Ref);
            // TODO: validate procedure
            targetPosition = UINT16_MAX;
            calibrationReqeusted = true;
            break;
        case SERVO_SENSOR_REFRESH_DIVIDER:
            refreshDivider = data;
            refreshCounter = 0;
            break;
        case SERVO_POSITION:
        case SERVO_POSITION_RAW:
            return -2;
        default:
            return -1;
    }
    return 0;
}

int ServoChannel::getVariable(uint8_t variableId, int32_t &data) const
{
    switch (variableId) {
        case SERVO_TARGET_POSITION:
            data = targetPosition;
            break;
        case SERVO_POSITION_STARTPOINT:
            data = tPosFromCanonic(pwmRef.start, pwmRef);
            break;
        case SERVO_POSITION_ENDPOINT:
            data = tPosFromCanonic(pwmRef.end, pwmRef);
            break;
        case SERVO_SENSOR_REFRESH_DIVIDER:
            data = static_cast<int32_t>(refreshDivider);
            break;
        case SERVO_POSITION:
            data = feedbackPosition;
            break;
        case SERVO_POSITION_RAW:
            data = *feedbackMeasurement << 4;
            break;
        default:
            return -1;
    }
    return 0;
}

// TODO: check if conversion is needed
uint16_t ServoChannel::tPosToCanonic(const uint16_t pos, const ServoRefPos &ref)
{
    if (ref.end == ref.start)
    {
        return UINT16_MAX;
    }

    // check if servo is reversed
    if (ref.end < ref.start)
    {
        // check if out of bounds
        if (pos <= ref.end)
        {
            return UINT16_MAX;
        }
        if (pos >= ref.start)
        {
            return 0;
        }
        return UINT16_MAX - (pos - ref.end) * (UINT16_MAX / (ref.start - ref.end));
    }

    // check if out of bounds
    if (pos <= ref.end)
    {
        return 0;
    }
    if (pos >= ref.start)
    {
        return UINT16_MAX;
    }

    return (pos - ref.start) * (UINT16_MAX / (ref.end - ref.start));
}

// TODO: check if conversion is needed
uint16_t ServoChannel::tPosFromCanonic(const uint16_t pos, const ServoRefPos &ref)
{
    if (ref.end == ref.start)
    {
        return ref.end;
    }
    if (ref.end < ref.start)
    {
        // reverse servo
        return (UINT16_MAX - pos) / (UINT16_MAX / (ref.start - ref.end)) + ref.end;
    }
    return pos / (UINT16_MAX / (ref.start - ref.end)) + ref.start;
}

void ServoChannel::checkPositionStability(const bool reset)
{
    if (targetPosition < feedbackPosition
                ? feedbackPosition - targetPosition
                : targetPosition - feedbackPosition
                < POS_DEV)
    {
        positionStabilityCounter++;
    }
    else if (reset)
    {
        positionStabilityCounter = 0;
    }
}


