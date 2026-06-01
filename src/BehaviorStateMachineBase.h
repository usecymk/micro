#ifndef MICRO3D_BEHAVIOR_STATE_MACHINE_BASE_H
#define MICRO3D_BEHAVIOR_STATE_MACHINE_BASE_H

class BehaviorStateMachineBase
{
public:
    virtual ~BehaviorStateMachineBase() = default;

    void setRandomSeed(unsigned int seed)
    {
        randomState = seed == 0 ? 0xA341316Cu : seed;
    }

protected:
    unsigned int randomState = 0xA341316Cu;

    float nextRandom01()
    {
        randomState = randomState * 1664525u + 1013904223u;
        return (float)((randomState >> 8) & 0x00FFFFFFu) / 16777215.0f;
    }
};

#endif // MICRO3D_BEHAVIOR_STATE_MACHINE_BASE_H
