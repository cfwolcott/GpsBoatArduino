// RunningAverage.h
// Provides functions to do Running Average computations
// Several objects of the defined struct below can be used for several running average types
// Probably should make this a C++ someday!
//
// Global Function Prototypes
//
#define RA_MAX_SIGNED_SAMPLES   20

// time smoothing constant for low-pass filter
// 0 ? alpha ? 1 ; a smaller value basically means more smoothing
#define LOW_PASS_FILTER_RATE (0.20f)

typedef struct
{
	bool bInitialized;
	S16 s16Average;
	S32 s32Sum;
	U8  u8SampleIndex;
	U8  u8SampleCount;
	U8  u8MaxSamples;
	S16 asSamples[RA_MAX_SIGNED_SAMPLES];
} RUNNING_SIGNED_AVERAGE_TYPE;

// Running Average Functions

// For Running Average functions

int LowPassFilter( int s16LastValue, int s16CurrentValue );
void RA_Signed_Init( U8 u8SampleCount, RUNNING_SIGNED_AVERAGE_TYPE *ptSamples );
S16 RA_ComputeSingedAverage( S16 s16Sample, RUNNING_SIGNED_AVERAGE_TYPE *ptSamples );
