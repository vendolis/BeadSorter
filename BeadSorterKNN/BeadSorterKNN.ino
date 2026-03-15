#include <Arduino.h>
#include <Servo.h>
#include <AccelStepper.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <avr/pgmspace.h>

// ─── IntelliSense stubs (never compiled by Arduino toolchain) ───
#ifdef __INTELLISENSE__
  #define F(x) x
  #define PSTR(x) x
  #define snprintf_P snprintf
  typedef char __FlashStringHelper;
#endif

// ═══════════════════════════════════════════════════════════════════
//  Pin definitions & hardware constants  (unchanged from BeadSorter)
// ═══════════════════════════════════════════════════════════════════
#define dirPin 2
#define stepPin 3
#define motorInterfaceType 1

#define stepperMaxSpeed 6000
#define stepperAccel 9000

#define stepperStepsPerRot 200
#define stepperMicroStepping 8
#define numContainerSlots 16
#define stepperMulti (stepperStepsPerRot * stepperMicroStepping / numContainerSlots)

#define motorSpeed 255
#define GSM2 5
#define in3 7
#define in4 6
#define hopperMotorReverseTime 6000

#define setupPin 11

#define photoLEDPin 9
#define photoSensorPin A0
#define photoSensorThreshold 600
#define photoSensorCalibMinDiff 100

#define servoAngleIn 36
#define servoAngleOut 62
#define servoAngleWiggle 2
#define servoPin 8

#define SENSOR_INTEGRATION_TIME TCS34725_INTEGRATIONTIME_101MS
#define SENSOR_GAIN             TCS34725_GAIN_16X

// ═══════════════════════════════════════════════════════════════════
//  KNN classifier constants  (ported from classify_bead.py)
// ═══════════════════════════════════════════════════════════════════

// Null-scan baseline (empty chamber raw readings)
#define NULL_R 498.1f
#define NULL_G 372.2f
#define NULL_B 244.2f

// Sum threshold below which the reading is treated as empty chamber
#define NULL_SUM_THRESHOLD 50.0f

// Max corrected RGB value (for scaling to 0-1)
#define RGB_MAX 7931.9f

// Normalization parameters (mean, std) for each feature
#define NORM_H_SIN_MEAN 0.470533f
#define NORM_H_SIN_STD  0.356254f
#define NORM_H_COS_MEAN 0.488719f
#define NORM_H_COS_STD  0.642882f
#define NORM_S_MEAN     0.415330f
#define NORM_S_STD      0.189916f
#define NORM_L_MEAN     0.191847f
#define NORM_L_STD      0.165316f

#define K_NEIGHBORS 3
#define NUM_REPRESENTATIVES 190
#define NUM_CLUSTERS 38

// Maximum number of bead-color clusters that get their own container
#define MAX_SORT_CLUSTERS 11

// ─── Representative points stored in PROGMEM ───
// Each row: { h_sin, h_cos, s, l }   (original / un-normalised space)
static const float repFeatures[NUM_REPRESENTATIVES][4] PROGMEM = {
  {0.981397,0.191989,0.458726,0.240332},{0.981061,0.193701,0.460193,0.275317},
  {0.966849,0.255347,0.448422,0.228985},{0.964240,0.265030,0.453953,0.257856},
  {0.964255,0.264975,0.455514,0.185427},{0.992586,0.121547,0.350025,0.327574},
  {0.939883,-0.341498,0.265038,0.354364},{0.993824,0.110967,0.343485,0.360605},
  {0.988113,-0.153728,0.352519,0.313454},{0.992391,0.123125,0.342631,0.289752},
  {0.891929,0.452175,0.231817,0.381218},{0.959937,0.280216,0.227916,0.461589},
  {0.929334,0.369241,0.230818,0.440220},{0.929025,0.370016,0.227779,0.388530},
  {0.960432,0.278516,0.241869,0.412547},{0.920954,-0.389671,0.412970,0.079211},
  {0.972815,-0.231584,0.352991,0.100706},{0.951580,-0.307401,0.393046,0.091566},
  {0.987681,0.156480,0.333548,0.020965},{0.930853,-0.365393,0.320121,0.075996},
  {0.585068,-0.810984,0.380327,0.184110},{0.547077,-0.837082,0.417597,0.143074},
  {0.544982,-0.838448,0.393963,0.157257},{0.572035,-0.820230,0.379107,0.127504},
  {0.583186,-0.812339,0.404314,0.165704},{0.894990,-0.446087,0.142407,0.000857},
  {0.894990,-0.446087,0.124142,0.000983},{0.894990,-0.446087,0.142407,0.000857},
  {0.894990,-0.446087,0.142407,0.000857},{0.894990,-0.446087,0.142407,0.000857},
  {0.922234,-0.386632,0.074308,0.000794},{0.922234,-0.386632,0.074308,0.000794},
  {0.922234,-0.386632,0.074308,0.000794},{0.922234,-0.386632,0.074308,0.000794},
  {0.922234,-0.386632,0.074308,0.000794},{0.922234,-0.386632,0.064124,0.000920},
  {0.922234,-0.386632,0.064124,0.000920},{0.922234,-0.386632,0.064124,0.000920},
  {0.922234,-0.386632,0.064124,0.000920},{0.922234,-0.386632,0.064124,0.000920},
  {0.020072,-0.999799,0.159695,0.142822},{-0.022405,-0.999749,0.217004,0.109480},
  {-0.055008,-0.998486,0.198583,0.145347},{-0.063118,-0.998006,0.255538,0.118872},
  {-0.016961,-0.999856,0.219937,0.142700},{0.547524,0.836790,0.365807,0.457188},
  {0.567393,0.823447,0.367812,0.506230},{0.563056,0.826419,0.360422,0.482906},
  {0.612779,0.790254,0.364944,0.514614},{0.534926,0.844899,0.387775,0.510516},
  {0.459499,-0.888178,0.170173,0.201445},{0.405781,-0.913970,0.210897,0.177492},
  {0.298850,-0.954300,0.201876,0.184173},{0.321960,-0.946753,0.175123,0.165515},
  {0.448152,-0.893957,0.211814,0.185056},{0.424726,-0.905322,0.293819,0.218150},
  {0.352627,-0.935764,0.325650,0.203021},{0.351339,-0.936248,0.302311,0.183039},
  {0.386276,-0.922383,0.272756,0.186695},{0.391780,-0.920059,0.315312,0.233468},
  {0.286396,0.958111,0.248089,0.233535},{0.223224,0.974767,0.293115,0.240028},
  {0.329882,0.944022,0.269887,0.286611},{0.236568,0.971615,0.281036,0.205484},
  {0.245203,0.969472,0.285723,0.260136},{0.073528,0.997293,0.302607,0.103554},
  {0.221102,0.975251,0.342138,0.115909},{0.096842,0.995300,0.264220,0.141502},
  {0.080080,0.996788,0.283038,0.090001},{0.109854,0.993948,0.301340,0.107336},
  {0.834670,0.550750,0.136718,0.001435},{0.834670,0.550750,0.136718,0.001435},
  {0.834670,0.550750,0.136718,0.001435},{0.834670,0.550750,0.136718,0.001435},
  {0.834670,0.550750,0.136718,0.001435},{0.842571,0.538585,0.173035,0.001498},
  {0.842571,0.538585,0.159606,0.001624},{0.842571,0.538585,0.173035,0.001498},
  {0.842571,0.538585,0.173035,0.001498},{0.842571,0.538585,0.173035,0.001498},
  {0.180416,0.983590,0.576197,0.426237},{0.198222,0.980157,0.619281,0.525141},
  {0.164347,0.986403,0.582689,0.465193},{0.205122,0.978736,0.572196,0.489147},
  {0.183675,0.982987,0.580247,0.389802},{0.585336,0.810791,0.370477,0.160980},
  {0.558959,0.829195,0.346751,0.154361},{0.604400,0.796681,0.328827,0.155874},
  {0.512462,0.858710,0.346540,0.163186},{0.618807,0.785543,0.295347,0.160098},
  {0.589206,0.807983,0.742749,0.682920},{0.569490,0.821998,0.797881,0.679390},
  {0.526458,0.850201,0.701552,0.649385},{0.542635,0.839969,0.770344,0.676113},
  {0.516347,0.856379,0.754113,0.666216},{0.325406,0.945574,0.583481,0.176109},
  {0.401708,0.915768,0.518034,0.215759},{0.343133,0.939287,0.558201,0.222819},
  {0.340198,0.940354,0.578148,0.163123},{0.370227,0.928941,0.542378,0.194452},
  {0.246144,0.969233,0.606028,0.040266},{0.242986,0.970030,0.530091,0.065417},
  {0.405267,0.914199,0.504454,0.073990},{0.321788,0.946812,0.436952,0.061761},
  {0.290441,0.956893,0.563672,0.043291},{0.534942,0.844889,0.598884,0.618434},
  {0.546963,0.837156,0.458963,0.547708},{0.545818,0.837904,0.513269,0.581684},
  {0.385481,0.922716,0.475333,0.535037},{0.536604,0.843834,0.565092,0.586916},
  {0.052657,0.998613,0.452176,0.240910},{0.124668,0.992198,0.448826,0.254085},
  {0.043531,0.999052,0.446314,0.254526},{0.047209,0.998885,0.458264,0.219415},
  {0.046596,0.998914,0.450702,0.226034},{0.759067,0.651013,0.306043,0.016501},
  {0.662049,0.749461,0.331813,0.019779},{0.653512,0.756916,0.387653,0.026524},
  {0.687286,0.726387,0.275935,0.003453},{0.522947,0.852365,0.400540,0.023939},
  {0.416849,0.908976,0.505754,0.325694},{0.425735,0.904848,0.500649,0.410730},
  {0.377655,0.925946,0.513047,0.363453},{0.407424,0.913239,0.499438,0.366920},
  {0.414647,0.909982,0.488293,0.337986},{0.996914,0.078500,0.124142,0.000983},
  {0.999253,0.038640,0.166818,0.001487},{0.996914,0.078500,0.124142,0.000983},
  {0.996914,0.078500,0.124142,0.000983},{0.996914,0.078500,0.124142,0.000983},
  {0.413893,0.910325,0.628823,0.472064},{0.406666,0.913577,0.641547,0.410036},
  {0.403518,0.914972,0.654301,0.504024},{0.417795,0.908541,0.640063,0.496963},
  {0.400335,0.916369,0.645864,0.445652},{-0.013118,0.999914,0.545942,0.313965},
  {-0.043942,0.999034,0.577169,0.226534},{-0.032742,0.999464,0.584016,0.293604},
  {-0.037243,0.999306,0.566070,0.263158},{0.017823,0.999841,0.566576,0.293735},
  {0.973629,0.228138,0.129940,0.001424},{0.973629,0.228138,0.129940,0.001424},
  {0.973629,0.228138,0.129940,0.001424},{0.973629,0.228138,0.129940,0.001424},
  {0.973629,0.228138,0.129940,0.001424},{0.939693,0.342020,0.200696,0.001550},
  {0.939693,0.342020,0.200696,0.001550},{0.939693,0.342020,0.200696,0.001550},
  {0.939693,0.342020,0.200696,0.001550},{0.939693,0.342020,0.200696,0.001550},
  {0.953931,0.300027,0.166818,0.001487},{0.953931,0.300027,0.182269,0.001361},
  {0.953931,0.300027,0.153782,0.001613},{0.953931,0.300027,0.166818,0.001487},
  {0.953931,0.300027,0.166818,0.001487},{0.312846,0.949804,0.599774,0.377005},
  {0.350827,0.936440,0.599736,0.416970},{0.351294,0.936265,0.587963,0.392512},
  {0.317118,0.948386,0.610685,0.397933},{0.336983,0.941511,0.600233,0.392260},
  {0.457770,0.889071,0.601072,0.297012},{0.481217,0.876601,0.550659,0.328783},
  {0.451136,0.892455,0.594304,0.245134},{0.434161,0.900835,0.594206,0.371521},
  {0.448567,0.893749,0.600964,0.330106},{0.247651,0.968849,0.657120,0.218343},
  {0.276197,0.961101,0.657719,0.178946},{0.286856,0.957974,0.645284,0.233094},
  {0.285030,0.958519,0.647563,0.253202},{0.280709,0.959793,0.651651,0.198991},
  {0.160340,0.987062,0.705032,0.206366},{0.157148,0.987575,0.701567,0.256354},
  {0.167470,0.985877,0.687767,0.167095},{0.159085,0.987265,0.675279,0.241036},
  {0.092076,0.995752,0.676488,0.230635},{0.045995,0.998942,0.606987,0.200756},
  {0.034644,0.999400,0.577591,0.164699},{0.058538,0.998285,0.632807,0.172642},
  {0.020613,0.999788,0.597579,0.149697},{0.052296,0.998632,0.610163,0.190103},
  {0.065744,0.997837,0.696868,0.205799},{0.056129,0.998424,0.686954,0.206934},
  {0.095189,0.995459,0.702278,0.204034},{0.046068,0.998938,0.682099,0.193066},
  {0.057201,0.998363,0.705876,0.199243},{0.052411,0.998626,0.681783,0.167914},
  {0.057309,0.998356,0.714143,0.130471},{0.054503,0.998514,0.709559,0.144906},
  {0.063301,0.997994,0.697092,0.171822},{0.067936,0.997690,0.692531,0.163943},
};

// Cluster ID for each representative point (parallel to repFeatures)
static const uint8_t repCluster[NUM_REPRESENTATIVES] PROGMEM = {
   0, 0, 0, 0, 0,  1, 1, 1, 1, 1,  2, 2, 2, 2, 2,  3, 3, 3, 3, 3,
   4, 4, 4, 4, 4,  5, 5, 5, 5, 5,  6, 6, 6, 6, 6,  7, 7, 7, 7, 7,
   8, 8, 8, 8, 8,  9, 9, 9, 9, 9, 10,10,10,10,10, 11,11,11,11,11,
  12,12,12,12,12, 13,13,13,13,13, 14,14,14,14,14, 15,15,15,15,15,
  16,16,16,16,16, 17,17,17,17,17, 18,18,18,18,18, 19,19,19,19,19,
  20,20,20,20,20, 21,21,21,21,21, 22,22,22,22,22, 23,23,23,23,23,
  24,24,24,24,24, 25,25,25,25,25, 26,26,26,26,26, 27,27,27,27,27,
  28,28,28,28,28, 29,29,29,29,29, 30,30,30,30,30, 31,31,31,31,31,
  32,32,32,32,32, 33,33,33,33,33, 34,34,34,34,34, 35,35,35,35,35,
  36,36,36,36,36, 37,37,37,37,37,
};

// ═══════════════════════════════════════════════════════════════════
//  Global objects
// ═══════════════════════════════════════════════════════════════════
Adafruit_TCS34725 tcs = Adafruit_TCS34725(SENSOR_INTEGRATION_TIME, SENSOR_GAIN);
Servo servo;
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

uint16_t resultRaw[4]  = {0, 0, 0, 0};     // R, G, B, C raw sensor counts
uint16_t beadCounter = 0;

// Cluster-to-container mapping:  clusterContainer[clusterID] = container slot, or -1 if unassigned
int8_t clusterContainer[NUM_CLUSTERS];
uint8_t numAssignedClusters = 0;

// Physical container layout — same as original.
// -1 = empty, 666 = blocked (only 12 usable slots out of 16)
const int dynamicContainerArraySize = 16;
int dynamicContainerArray[dynamicContainerArraySize] = {
  -1, -1, 666, -1, -1, -1, 666, -1, -1, -1, 666, -1, -1, -1, 666, -1
};

// ═══════════════════════════════════════════════════════════════════
//  RGB → HLS conversion (port of Python _rgb_to_hls)
//  Input: r, g, b in [0..1]
//  Output: h in [0..1], l in [0..1], s in [0..1]
// ═══════════════════════════════════════════════════════════════════
void rgbToHLS(float r, float g, float b, float &h, float &l, float &s) {
  float maxC = max(r, max(g, b));
  float minC = min(r, min(g, b));
  l = (maxC + minC) / 2.0f;

  if (maxC == minC) {
    h = 0.0f;
    s = 0.0f;
    return;
  }

  float d = maxC - minC;
  s = (l > 0.5f) ? d / (2.0f - maxC - minC) : d / (maxC + minC);

  if (maxC == r) {
    h = (g - b) / d + ((g < b) ? 6.0f : 0.0f);
  } else if (maxC == g) {
    h = (b - r) / d + 2.0f;
  } else {
    h = (r - g) / d + 4.0f;
  }
  h /= 6.0f;
}

// ═══════════════════════════════════════════════════════════════════
//  KNN bead classifier (port of classify_bead.py)
//  Returns cluster ID (0-37), or -1 for empty chamber.
// ═══════════════════════════════════════════════════════════════════
int classifyBead(uint16_t rawR, uint16_t rawG, uint16_t rawB, uint16_t rawC) {
  // Subtract ambient baseline
  float rCorr = max((float)rawR - NULL_R, 1.0f);
  float gCorr = max((float)rawG - NULL_G, 1.0f);
  float bCorr = max((float)rawB - NULL_B, 1.0f);

  // Empty chamber check
  if (rCorr + gCorr + bCorr < NULL_SUM_THRESHOLD)
    return -1;

  // Scale to 0-1
  float r01 = rCorr / RGB_MAX;
  float g01 = gCorr / RGB_MAX;
  float b01 = bCorr / RGB_MAX;

  // Convert to HLS
  float h, l, s;
  rgbToHLS(r01, g01, b01, h, l, s);

  // Encode hue on unit circle
  float hRad = h * 2.0f * (float)M_PI;
  float hSin = sinf(hRad);
  float hCos = cosf(hRad);

  // Normalize features
  float hSinN = (hSin - NORM_H_SIN_MEAN) / NORM_H_SIN_STD;
  float hCosN = (hCos - NORM_H_COS_MEAN) / NORM_H_COS_STD;
  float sN    = (s    - NORM_S_MEAN)      / NORM_S_STD;
  float lN    = (l    - NORM_L_MEAN)      / NORM_L_STD;

  // Find K nearest neighbors — keep top 3 by distance
  float   bestDist[K_NEIGHBORS];
  uint8_t bestCluster[K_NEIGHBORS];
  for (int i = 0; i < K_NEIGHBORS; i++) {
    bestDist[i] = 1e18f;
    bestCluster[i] = 0;
  }

  for (int i = 0; i < NUM_REPRESENTATIVES; i++) {
    // Read representative features from PROGMEM
    float repHSin = pgm_read_float_near(&repFeatures[i][0]);
    float repHCos = pgm_read_float_near(&repFeatures[i][1]);
    float repS    = pgm_read_float_near(&repFeatures[i][2]);
    float repL    = pgm_read_float_near(&repFeatures[i][3]);

    // Normalize representative
    float rhSinN = (repHSin - NORM_H_SIN_MEAN) / NORM_H_SIN_STD;
    float rhCosN = (repHCos - NORM_H_COS_MEAN) / NORM_H_COS_STD;
    float rsN    = (repS    - NORM_S_MEAN)      / NORM_S_STD;
    float rlN    = (repL    - NORM_L_MEAN)      / NORM_L_STD;

    float d0 = hSinN - rhSinN;
    float d1 = hCosN - rhCosN;
    float d2 = sN    - rsN;
    float d3 = lN    - rlN;
    float dist = d0*d0 + d1*d1 + d2*d2 + d3*d3;  // squared distance (skip sqrt for comparison)

    // Insert into top-K if closer
    if (dist < bestDist[K_NEIGHBORS - 1]) {
      bestDist[K_NEIGHBORS - 1]    = dist;
      bestCluster[K_NEIGHBORS - 1] = pgm_read_byte_near(&repCluster[i]);
      // Bubble up
      for (int j = K_NEIGHBORS - 1; j > 0; j--) {
        if (bestDist[j] < bestDist[j - 1]) {
          float   tmpD = bestDist[j];    bestDist[j]    = bestDist[j-1];    bestDist[j-1]    = tmpD;
          uint8_t tmpC = bestCluster[j]; bestCluster[j] = bestCluster[j-1]; bestCluster[j-1] = tmpC;
        } else break;
      }
    }
  }

  // Distance-weighted vote (use actual Euclidean distance for weighting)
  // votes array: cluster_id -> accumulated weight.  Only K_NEIGHBORS entries matter.
  float voteWeight[NUM_CLUSTERS];
  memset(voteWeight, 0, sizeof(voteWeight));

  for (int i = 0; i < K_NEIGHBORS; i++) {
    float dist = sqrtf(bestDist[i]);
    float weight = 1.0f / (dist + 1e-9f);
    voteWeight[bestCluster[i]] += weight;
  }

  // Find cluster with highest vote
  int bestVoteCluster = 0;
  float bestVoteWeight = 0.0f;
  for (int i = 0; i < NUM_CLUSTERS; i++) {
    if (voteWeight[i] > bestVoteWeight) {
      bestVoteWeight = voteWeight[i];
      bestVoteCluster = i;
    }
  }

  return bestVoteCluster;
}

// ═══════════════════════════════════════════════════════════════════
//  Container allocation helpers
// ═══════════════════════════════════════════════════════════════════

// Find the next free (== -1) slot in dynamicContainerArray, skipping blocked (666) slots.
// Returns the slot index, or -1 if all full.
int getNextFreeContainerSlot() {
  for (int i = 0; i < dynamicContainerArraySize - 1; i++) {  // slot 15 reserved
    if (dynamicContainerArray[i] == -1)
      return i;
  }
  return -1;
}

// Assign a cluster to a physical container slot.  Returns the slot, or 15 if full.
int assignClusterToContainer(int clusterId) {
  // Already assigned?
  if (clusterContainer[clusterId] >= 0)
    return clusterContainer[clusterId];

  // Can we still assign new clusters?
  if (numAssignedClusters >= MAX_SORT_CLUSTERS)
    return 15;

  int slot = getNextFreeContainerSlot();
  if (slot < 0)
    return 15;

  clusterContainer[clusterId] = (int8_t)slot;
  dynamicContainerArray[slot] = clusterId;
  numAssignedClusters++;

  Serial.print(F("Assigned cluster ")); Serial.print(clusterId);
  Serial.print(F(" -> container slot ")); Serial.println(slot);
  return slot;
}

// ═══════════════════════════════════════════════════════════════════
//  Hardware helpers (identical to BeadSorter.ino)
// ═══════════════════════════════════════════════════════════════════

unsigned long timediff(unsigned long t1, unsigned long t2) {
  signed long d = (signed long)t1 - (signed long)t2;
  if (d < 0) d = -d;
  return (unsigned long)d;
}

void calibratePhotoSensor() {
  Serial.println(F("[INIT] Calibrating photo sensor..."));
  int sumOn = 0, sumOff = 0;
  for (int i = 0; i < 3; i++) {
    digitalWrite(photoLEDPin, HIGH); delay(50);
    sumOn += analogRead(photoSensorPin);
    digitalWrite(photoLEDPin, LOW);  delay(50);
    sumOff += analogRead(photoSensorPin);
  }
  int avgOn = sumOn / 3, avgOff = sumOff / 3;
  digitalWrite(photoLEDPin, HIGH);

  Serial.print(F("Photo sensor calib: LED on=")); Serial.print(avgOn);
  Serial.print(F(", LED off=")); Serial.print(avgOff);
  Serial.print(F(", diff=")); Serial.println(abs(avgOn - avgOff));

  if (abs(avgOn - avgOff) < photoSensorCalibMinDiff) {
    Serial.println(F("ERROR: Photo sensor calib failed!"));
    while (1);
  }
  Serial.println(F("Photo sensor calibration OK."));
}

void handleHopperMotor(bool successfullBead) {
  static unsigned long timestamp = 0;
  static bool direction = false;

  if (!successfullBead) {
    if (timediff(timestamp, millis()) > hopperMotorReverseTime) {
      direction = !direction;
      timestamp = millis();
    }
  } else {
    timestamp = millis();
  }

  int photoSensor = analogRead(photoSensorPin);
  if (photoSensor > photoSensorThreshold) {
    startHopperMotor(direction);
  } else {
    stopHopperMotor();
    Serial.print(F("Photosensor detected Beads ")); Serial.println(photoSensor);
  }
}

void servoFeedIn() {
  int low  = servoAngleIn - servoAngleWiggle;
  int high = servoAngleIn + servoAngleWiggle;
  for (int i = low; i < high; i++) { servo.write(i); delay(100); }
}

void servoWiggleIn() {
  int low  = servoAngleIn - servoAngleWiggle * 2;
  int high = servoAngleIn + servoAngleWiggle * 2;
  for (int c = 0; c < 3; c++) {
    servo.write(low);  delay(100);
    servo.write(high); delay(100);
  }
  servo.write(servoAngleIn); delay(100);
}

void servoFeedOut() {
  int low  = servoAngleOut - servoAngleWiggle;
  int high = servoAngleOut + servoAngleWiggle;
  for (int count = 0; count < 3; count++) {
    for (int i = low; i < high; i++)  { servo.write(i); delay(100); }
    for (int i = high; i > low; i--)  { servo.write(i); delay(50);  }
  }
  delay(200);
}

void readColorSensor() {
  delay(200);
  tcs.getRawData(&resultRaw[0], &resultRaw[1], &resultRaw[2], &resultRaw[3]);
}

void moveSorterToPosition(int position) {
  int currentPos = stepper.currentPosition() / stepperMulti;
  int diffToPosition = position - currentPos;

  if (diffToPosition > 8)
    position = currentPos + diffToPosition - 16;
  else if (diffToPosition < -8)
    position = currentPos + diffToPosition + 16;

  position *= stepperMulti;
  stepper.moveTo(position);
  stepper.runToPosition();
  stepper.setCurrentPosition(stepper.currentPosition() % (16 * stepperMulti));
}

void stopHopperMotor() {
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(GSM2, motorSpeed);
}

void startHopperMotor(bool dir) {
  if (dir) { digitalWrite(in3, HIGH); digitalWrite(in4, LOW); }
  else     { digitalWrite(in3, LOW);  digitalWrite(in4, HIGH); }
  analogWrite(GSM2, motorSpeed);
}

void printRawReading() {
  bool sat = (resultRaw[0] >= 65530 || resultRaw[1] >= 65530 ||
              resultRaw[2] >= 65530 || resultRaw[3] >= 65530);
  Serial.print(F("  raw  R=")); Serial.print(resultRaw[0]);
  Serial.print(F(" G="));       Serial.print(resultRaw[1]);
  Serial.print(F(" B="));       Serial.print(resultRaw[2]);
  Serial.print(F(" C="));       Serial.print(resultRaw[3]);
  Serial.println(sat ? F(" [SATURATED]") : F(""));
}

void printTables() {
  char line[48];
  Serial.println(F("Cluster -> Container mapping:"));
  for (int i = 0; i < NUM_CLUSTERS; i++) {
    if (clusterContainer[i] >= 0) {
      snprintf_P(line, sizeof(line), PSTR("  Cluster %2d -> Slot %2d"), i, clusterContainer[i]);
      Serial.println(line);
    }
  }
  Serial.println(F("Container array:"));
  for (int i = 0; i < dynamicContainerArraySize; i++) {
    Serial.print(dynamicContainerArray[i]);
    Serial.print(' ');
  }
  Serial.println();
}

// ═══════════════════════════════════════════════════════════════════
//  Setup & Loop
// ═══════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println(F("=== BeadSorterKNN starting ==="));

  // Initialise cluster mapping — all unassigned
  memset(clusterContainer, -1, sizeof(clusterContainer));

  Serial.println(F("[INIT] Hopper motor pins..."));
  pinMode(GSM2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  Serial.println(F("[INIT] Setup button pin..."));
  pinMode(setupPin, INPUT);

  Serial.println(F("[INIT] Photo sensor LED pin..."));
  pinMode(photoLEDPin, OUTPUT);
  digitalWrite(photoLEDPin, HIGH);

  Serial.println(F("[INIT] Servo..."));
  servo.attach(servoPin);
  servo.write(servoAngleIn);

  Serial.print(F("[INIT] Stepper (maxSpeed=")); Serial.print(stepperMaxSpeed);
  Serial.print(F(", accel="));                  Serial.print(stepperAccel);
  Serial.println(F(")..."));
  stepper.setMaxSpeed(stepperMaxSpeed);
  stepper.setAcceleration(stepperAccel);
  stepper.setCurrentPosition(0);

  calibratePhotoSensor();

  if (tcs.begin()) {
    Serial.println(F("[INIT] TCS34725 color sensor found."));
  } else {
    Serial.println(F("ERROR: TCS34725 not found, exiting!"));
    while (1);
  }

  Serial.println(F("=== Setup complete. KNN classifier active. ==="));
  Serial.println();
}

void loop() {
  static bool successfullBead = false;

  // Serial command: 't' prints tables
  while (Serial.available()) {
    int c = Serial.read();
    switch (c) {
      case 't': case 'T': printTables(); break;
      default: break;
    }
  }

  handleHopperMotor(successfullBead);

  // Button press pauses/resumes scanning
  if (digitalRead(setupPin) == HIGH) {
    while (digitalRead(setupPin) == HIGH) {}
    delay(50);
    stopHopperMotor();
    Serial.println(F("=== PAUSED — press button to resume ==="));
    while (digitalRead(setupPin) == LOW) {}
    while (digitalRead(setupPin) == HIGH) {}
    delay(50);
    Serial.println(F("=== RESUMED ==="));
  }

  servoFeedIn();
  delay(500);
  servoWiggleIn();
  delay(200);
  readColorSensor();

  // Classify via KNN
  int cluster = classifyBead(resultRaw[0], resultRaw[1], resultRaw[2], resultRaw[3]);

  if (cluster == -1) {
    // Null scan — no bead present
    Serial.println(F("No bead detected (null scan):"));
    printRawReading();
    moveSorterToPosition(15);
    successfullBead = false;
  } else {
    // Bead detected
    Serial.println();
    Serial.print(F("Bead #")); Serial.print(beadCounter++);
    Serial.print(F("  -> Cluster ")); Serial.println(cluster);
    printRawReading();

    int containerSlot = assignClusterToContainer(cluster);
    Serial.print(F("  -> Container slot ")); Serial.println(containerSlot);
    moveSorterToPosition(containerSlot);
    successfullBead = true;
  }

  servoFeedOut();
}
