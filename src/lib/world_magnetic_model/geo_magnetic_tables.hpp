/****************************************************************************
 *
 *   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <stdint.h>

static constexpr float SAMPLING_RES = 10;
static constexpr float SAMPLING_MIN_LAT = -90;
static constexpr float SAMPLING_MAX_LAT = 90;
static constexpr float SAMPLING_MIN_LON = -180;
static constexpr float SAMPLING_MAX_LON = 180;

static constexpr int LAT_DIM = 19;
static constexpr int LON_DIM = 37;


// *INDENT-OFF*
// Magnetic declination data in radians * 10^-4
// Model: WMM-2020,
// Version: 0.5.1.11,
// Date: 2021.8164,
static constexpr const int16_t declination_table[19][37] {
	//    LONGITUDE:   -180,  -170,  -160,  -150,  -140,  -130,  -120,  -110,  -100,   -90,   -80,   -70,   -60,   -50,   -40,   -30,   -20,   -10,     0,    10,    20,    30,    40,    50,    60,    70,    80,    90,   100,   110,   120,   130,   140,   150,   160,   170,   180,
	/* LAT: -90 */ {  26005, 24260, 22514, 20769, 19024, 17278, 15533, 13788, 12042, 10297,  8552,  6807,  5061,  3316,  1571,  -175, -1920, -3665, -5410, -7156, -8901,-10646,-12392,-14137,-15882,-17628,-19373,-21118,-22864,-24609,-26354,-28100,-29845, 31241, 29496, 27751, 26005, },
	/* LAT: -80 */ {  22574, 20439, 18497, 16721, 15080, 13540, 12076, 10664,  9288,  7938,  6606,  5287,  3978,  2673,  1365,    47, -1294, -2664, -4073, -5523, -7016, -8551,-10127,-11744,-13407,-15124,-16908,-18779,-20760,-22877,-25150,-27583,-30153, 30038, 27417, 24908, 22574, },
	/* LAT: -70 */ {  14970, 13574, 12450, 11491, 10624,  9795,  8955,  8069,  7117,  6099,  5029,  3935,  2849,  1792,   769,  -241, -1277, -2380, -3577, -4874, -6251, -7678, -9123,-10564,-11989,-13404,-14830,-16314,-17933,-19839,-22347,-26128, 30786, 24176, 19626, 16843, 14970, },
	/* LAT: -60 */ {   8393,  8153,  7878,  7608,  7360,  7110,  6805,  6376,  5764,  4946,  3947,  2836,  1714,   682,  -204,  -966, -1695, -2514, -3511, -4701, -6025, -7391, -8710, -9920,-10986,-11886,-12604,-13099,-13271,-12822,-10729, -3567,  4821,  7600,  8386,  8521,  8393, },
	/* LAT: -50 */ {   5464,  5503,  5451,  5365,  5295,  5262,  5230,  5106,  4766,  4107,  3101,  1830,   488,  -699, -1579, -2152, -2553, -2991, -3666, -4666, -5899, -7171, -8311, -9216, -9820,-10066, -9880, -9130, -7628, -5271, -2370,   376,  2485,  3909,  4783,  5258,  5464, },
	/* LAT: -40 */ {   3939,  4033,  4044,  4003,  3947,  3916,  3923,  3915,  3744,  3215,  2199,   762,  -804, -2126, -3001, -3464, -3656, -3720, -3881, -4431, -5391, -6460, -7348, -7895, -8012, -7646, -6766, -5386, -3672, -1966,  -503,   719,  1762,  2625,  3279,  3708,  3939, },
	/* LAT: -30 */ {   2970,  3057,  3091,  3081,  3027,  2951,  2894,  2862,  2737,  2265,  1238,  -274, -1883, -3146, -3898, -4254, -4343, -4145, -3714, -3473, -3812, -4550, -5253, -5613, -5499, -4923, -3970, -2762, -1547,  -597,    84,   685,  1307,  1906,  2413,  2772,  2970, },
	/* LAT: -20 */ {   2328,  2375,  2397,  2404,  2365,  2275,  2170,  2096,  1951,  1461,   415, -1067, -2548, -3620, -4167, -4295, -4103, -3557, -2700, -1891, -1594, -1934, -2582, -3059, -3105, -2754, -2122, -1303,  -521,   -27,   239,   542,   980,  1454,  1872,  2177,  2328, },
	/* LAT: -10 */ {   1934,  1929,  1912,  1914,  1891,  1810,  1705,  1616,  1433,   887,  -166, -1539, -2817, -3659, -3942, -3728, -3162, -2385, -1539,  -778,  -302,  -322,  -785, -1293, -1513, -1428, -1117,  -617,  -115,   131,   184,   345,   713,  1145,  1534,  1817,  1934, },
	/* LAT:   0 */ {   1721,  1689,  1638,  1634,  1627,  1562,  1463,  1353,  1101,   483,  -552, -1778, -2839, -3438, -3453, -2965, -2202, -1419,  -763,  -224,   194,   314,    43,  -374,  -633,  -692,  -593,  -330,   -37,    57,     0,    90,   428,   864,  1279,  1595,  1721, },
	/* LAT:  10 */ {   1587,  1596,  1558,  1578,  1608,  1563,  1447,  1265,   891,   175,  -844, -1922, -2756, -3109, -2918, -2321, -1546,  -831,  -311,    72,   399,   553,   398,    76,  -161,  -269,  -291,  -208,  -101,  -139,  -275,  -247,    51,   500,   976,  1380,  1587, },
	/* LAT:  20 */ {   1408,  1557,  1621,  1716,  1805,  1788,  1639,  1341,   798,   -58, -1101, -2057, -2664, -2784, -2466, -1870, -1160,  -507,   -43,   270,   530,   680,   593,   351,   150,    32,   -57,  -118,  -195,  -382,  -617,  -675,  -444,    -1,   536,  1052,  1408, },
	/* LAT:  30 */ {   1113,  1480,  1741,  1967,  2128,  2141,  1958,  1539,   813,  -216, -1338, -2223, -2648, -2595, -2202, -1625,  -970,  -351,   116,   425,   658,   808,   791,   643,   493,   369,   215,     9,  -270,  -643, -1011, -1171, -1014,  -592,   -17,   592,  1113, },
	/* LAT:  40 */ {    762,  1349,  1844,  2236,  2486,  2531,  2317,  1786,   876,  -354, -1600, -2476, -2807, -2661, -2218, -1628,  -974,  -343,   170,   538,   811,  1013,  1111,  1102,  1026,   878,   614,   209,  -325,  -930, -1452, -1699, -1586, -1171,  -573,   101,   762, },
	/* LAT:  50 */ {    480,  1229,  1909,  2465,  2831,  2937,  2708,  2054,   910,  -594, -2027, -2952, -3253, -3065, -2577, -1931, -1221,  -521,    98,   607,  1029,  1387,  1671,  1844,  1866,  1682,  1243,   542,  -346, -1249, -1931, -2223, -2102, -1661, -1023,  -287,   480, },
	/* LAT:  60 */ {    296,  1151,  1957,  2650,  3152,  3362,  3145,  2328,   804, -1171, -2902, -3890, -4145, -3885, -3311, -2564, -1736,  -892,   -81,   673,  1368,  2002,  2548,  2948,  3113,  2933,  2305,  1199,  -210, -1532, -2409, -2724, -2560, -2064, -1365,  -558,   296, },
	/* LAT:  70 */ {     83,  1023,  1921,  2720,  3332,  3621,  3360,  2206,   -69, -2819, -4783, -5605, -5606, -5115, -4339, -3399, -2370, -1300,  -222,   842,  1870,  2839,  3710,  4421,  4875,  4916,  4321,  2868,   689, -1420, -2727, -3176, -3017, -2481, -1726,  -849,    83, },
	/* LAT:  80 */ {   -550,   375,  1231,  1924,  2307,  2109,   859, -1866, -5178, -7306, -8026, -7868, -7217, -6285, -5182, -3974, -2701, -1389,   -60,  1271,  2588,  3872,  5101,  6240,  7237,  7998,  8346,  7917,  6042,  2367, -1189, -2923, -3308, -2992, -2322, -1474,  -550, },
	/* LAT:  90 */ { -30114,-28368,-26623,-24877,-23132,-21387,-19641,-17896,-16151,-14405,-12660,-10915, -9170, -7425, -5679, -3934, -2189,  -444,  1301,  3046,  4792,  6537,  8282, 10027, 11773, 13518, 15263, 17009, 18754, 20500, 22245, 23991, 25736, 27482, 29227, 30973,-30114, },
};

// Magnetic inclination data in radians * 10^-4
// Model: WMM-2020,
// Version: 0.5.1.11,
// Date: 2021.8164,
static constexpr const int16_t inclination_table[19][37] {
	//    LONGITUDE:   -180,  -170,  -160,  -150,  -140,  -130,  -120,  -110,  -100,   -90,   -80,   -70,   -60,   -50,   -40,   -30,   -20,   -10,     0,    10,    20,    30,    40,    50,    60,    70,    80,    90,   100,   110,   120,   130,   140,   150,   160,   170,   180,
	/* LAT: -90 */ { -12576,-12576,-12576,-12576,-12576,-12576,-12576,-12576,-12576,-12576,-12576,-12576,-12576,-12576,-12576,-12576,-12576,-12576,-12576,-12576,-12576,-12576,-12576,-12576,-12576,-12576,-12576,-12576,-12576,-12576,-12576,-12576,-12576,-12576,-12576,-12576,-12576, },
	/* LAT: -80 */ { -13662,-13529,-13368,-13187,-12993,-12792,-12587,-12387,-12195,-12019,-11862,-11727,-11617,-11531,-11467,-11426,-11406,-11409,-11436,-11490,-11574,-11688,-11832,-12006,-12203,-12420,-12649,-12882,-13110,-13323,-13510,-13660,-13765,-13817,-13814,-13760,-13662, },
	/* LAT: -70 */ { -14112,-13793,-13474,-13151,-12818,-12474,-12118,-11760,-11416,-11107,-10853,-10668,-10555,-10503,-10489,-10493,-10502,-10512,-10536,-10593,-10700,-10874,-11119,-11432,-11804,-12220,-12668,-13132,-13598,-14051,-14469,-14816,-15007,-14955,-14725,-14427,-14112, },
	/* LAT: -60 */ { -13522,-13169,-12831,-12499,-12155,-11783,-11367,-10912,-10444,-10012, -9679, -9501, -9496, -9631, -9836,-10031,-10160,-10207,-10197,-10184,-10230,-10387,-10673,-11078,-11574,-12129,-12714,-13310,-13899,-14460,-14957,-15247,-15075,-14692,-14287,-13894,-13522, },
	/* LAT: -50 */ { -12497,-12156,-11826,-11504,-11181,-10834,-10435, -9963, -9432, -8909, -8515, -8385, -8580, -9038, -9603,-10118,-10484,-10653,-10630,-10483,-10340,-10342,-10557,-10968,-11506,-12097,-12684,-13224,-13672,-13971,-14079,-14007,-13804,-13521,-13193,-12846,-12497, },
	/* LAT: -40 */ { -11240,-10893,-10547,-10203, -9863, -9525, -9163, -8737, -8215, -7648, -7217, -7166, -7618, -8437, -9359,-10192,-10857,-11299,-11450,-11292,-10953,-10677,-10662,-10936,-11394,-11899,-12347,-12672,-12833,-12839,-12749,-12611,-12431,-12198,-11912,-11585,-11240, },
	/* LAT: -30 */ {  -9601, -9226, -8849, -8460, -8067, -7691, -7333, -6940, -6426, -5813, -5355, -5426, -6182, -7378, -8630, -9730,-10649,-11367,-11777,-11780,-11417,-10910,-10570,-10562,-10810,-11131,-11390,-11507,-11447,-11271,-11095,-10959,-10811,-10603,-10321, -9977, -9601, },
	/* LAT: -20 */ {  -7370, -6935, -6522, -6095, -5651, -5220, -4827, -4410, -3842, -3155, -2699, -2943, -4037, -5641, -7272, -8649, -9723,-10508,-10957,-11003,-10649,-10040, -9479, -9229, -9273, -9435, -9578, -9599, -9429, -9156, -8960, -8869, -8761, -8552, -8235, -7825, -7370, },
	/* LAT: -10 */ {  -4413, -3885, -3437, -3003, -2545, -2094, -1677, -1218,  -587,   120,   492,    82, -1229, -3138, -5116, -6736, -7845, -8490, -8764, -8710, -8309, -7631, -6967, -6617, -6575, -6667, -6786, -6812, -6626, -6329, -6171, -6174, -6128, -5909, -5525, -5000, -4413, },
	/* LAT:   0 */ {   -904,  -289,   169,   569,   986,  1403,  1796,  2246,  2837,  3420,  3631,  3150,  1871,   -30, -2076, -3739, -4763, -5203, -5269, -5105, -4672, -3963, -3258, -2883, -2821, -2892, -3018, -3093, -2964, -2727, -2668, -2804, -2858, -2668, -2244, -1618,  -904, },
	/* LAT:  10 */ {   2564,  3181,  3610,  3948,  4302,  4669,  5024,  5417,  5875,  6251,  6295,  5817,  4747,  3189,  1498,   113,  -705,  -964,  -879,  -647,  -241,   391,  1024,  1364,  1426,  1379,  1277,  1185,  1229,  1334,  1263,  1004,   820,   896,  1248,  1846,  2564, },
	/* LAT:  20 */ {   5418,  5940,  6315,  6609,  6921,  7265,  7611,  7963,  8302,  8507,  8425,  7969,  7141,  6050,  4924,  4009,  3471,  3344,  3489,  3729,  4056,  4518,  4977,  5233,  5289,  5268,  5219,  5162,  5152,  5136,  4966,  4639,  4347,  4261,  4432,  4852,  5418, },
	/* LAT:  30 */ {   7569,  7939,  8255,  8536,  8845,  9193,  9552,  9897, 10177, 10295, 10154,  9738,  9112,  8402,  7740,  7225,  6928,  6882,  7020,  7229,  7473,  7770,  8058,  8232,  8288,  8298,  8299,  8290,  8269,  8189,  7970,  7621,  7269,  7051,  7035,  7228,  7569, },
	/* LAT:  40 */ {   9266,  9486,  9741, 10026, 10353, 10715, 11085, 11426, 11681, 11766, 11619, 11261, 10789, 10315,  9918,  9632,  9480,  9473,  9580,  9740,  9913, 10096, 10267, 10392, 10469, 10528, 10581, 10614, 10599, 10491, 10252,  9904,  9539,  9255,  9112,  9124,  9266, },
	/* LAT:  50 */ {  10801, 10923, 11125, 11395, 11719, 12074, 12430, 12748, 12972, 13033, 12895, 12598, 12233, 11886, 11610, 11421, 11326, 11321, 11385, 11486, 11598, 11713, 11829, 11944, 12062, 12183, 12296, 12368, 12361, 12238, 11994, 11668, 11329, 11044, 10854, 10774, 10801, },
	/* LAT:  60 */ {  12318, 12392, 12544, 12763, 13035, 13336, 13640, 13907, 14084, 14112, 13976, 13726, 13436, 13164, 12943, 12785, 12693, 12660, 12674, 12721, 12788, 12872, 12977, 13108, 13267, 13441, 13606, 13716, 13724, 13606, 13383, 13104, 12825, 12588, 12418, 12327, 12318, },
	/* LAT:  70 */ {  13759, 13802, 13899, 14043, 14223, 14428, 14638, 14820, 14924, 14903, 14764, 14561, 14342, 14137, 13963, 13828, 13733, 13678, 13658, 13670, 13711, 13782, 13885, 14020, 14186, 14373, 14557, 14699, 14747, 14676, 14516, 14318, 14123, 13958, 13838, 13770, 13759, },
	/* LAT:  80 */ {  15000, 15014, 15053, 15113, 15191, 15277, 15357, 15402, 15382, 15300, 15186, 15060, 14937, 14822, 14722, 14640, 14577, 14536, 14517, 14521, 14547, 14596, 14667, 14759, 14871, 14997, 15132, 15263, 15370, 15416, 15379, 15294, 15200, 15116, 15053, 15013, 15000, },
	/* LAT:  90 */ {  15393, 15393, 15393, 15393, 15393, 15393, 15393, 15393, 15393, 15393, 15393, 15393, 15393, 15393, 15393, 15393, 15393, 15393, 15393, 15393, 15393, 15393, 15393, 15393, 15393, 15393, 15393, 15393, 15393, 15393, 15393, 15393, 15393, 15393, 15393, 15393, 15393, },
};

// Magnetic strength data in milli-Gauss * 10
// Model: WMM-2020,
// Version: 0.5.1.11,
// Date: 2021.8164,
static constexpr const int16_t strength_table[19][37] {
	//    LONGITUDE:  -180, -170, -160, -150, -140, -130, -120, -110, -100,  -90,  -80,  -70,  -60,  -50,  -40,  -30,  -20,  -10,    0,   10,   20,   30,   40,   50,   60,   70,   80,   90,  100,  110,  120,  130,  140,  150,  160,  170,  180,
	/* LAT: -90 */ {  5456, 5456, 5456, 5456, 5456, 5456, 5456, 5456, 5456, 5456, 5456, 5456, 5456, 5456, 5456, 5456, 5456, 5456, 5456, 5456, 5456, 5456, 5456, 5456, 5456, 5456, 5456, 5456, 5456, 5456, 5456, 5456, 5456, 5456, 5456, 5456, 5456, },
	/* LAT: -80 */ {  6062, 5999, 5920, 5829, 5726, 5615, 5497, 5375, 5253, 5133, 5019, 4913, 4819, 4740, 4676, 4631, 4607, 4604, 4625, 4671, 4741, 4834, 4949, 5080, 5223, 5373, 5523, 5666, 5797, 5911, 6003, 6073, 6117, 6137, 6133, 6108, 6062, },
	/* LAT: -70 */ {  6306, 6174, 6025, 5861, 5683, 5491, 5286, 5071, 4852, 4636, 4433, 4251, 4092, 3961, 3857, 3780, 3732, 3720, 3747, 3823, 3950, 4129, 4357, 4624, 4918, 5225, 5529, 5813, 6063, 6267, 6418, 6514, 6557, 6550, 6501, 6417, 6306, },
	/* LAT: -60 */ {  6191, 6000, 5800, 5592, 5374, 5139, 4883, 4605, 4312, 4022, 3755, 3527, 3346, 3212, 3112, 3039, 2988, 2968, 2993, 3083, 3250, 3500, 3825, 4210, 4632, 5065, 5487, 5871, 6196, 6445, 6610, 6689, 6691, 6629, 6517, 6367, 6191, },
	/* LAT: -50 */ {  5848, 5619, 5387, 5158, 4926, 4681, 4411, 4108, 3779, 3447, 3145, 2904, 2741, 2649, 2601, 2567, 2534, 2508, 2513, 2584, 2755, 3043, 3437, 3906, 4412, 4918, 5394, 5813, 6153, 6395, 6534, 6576, 6534, 6425, 6264, 6067, 5848, },
	/* LAT: -40 */ {  5396, 5151, 4907, 4668, 4434, 4197, 3943, 3659, 3346, 3019, 2720, 2496, 2378, 2351, 2370, 2393, 2398, 2388, 2376, 2404, 2531, 2802, 3216, 3728, 4274, 4799, 5267, 5657, 5950, 6138, 6227, 6231, 6162, 6032, 5851, 5634, 5396, },
	/* LAT: -30 */ {  4880, 4640, 4402, 4168, 3943, 3724, 3505, 3274, 3019, 2744, 2486, 2302, 2230, 2254, 2321, 2393, 2459, 2511, 2534, 2547, 2612, 2806, 3163, 3648, 4179, 4677, 5098, 5417, 5621, 5721, 5748, 5721, 5642, 5511, 5332, 5116, 4880, },
	/* LAT: -20 */ {  4322, 4110, 3903, 3698, 3503, 3320, 3152, 2989, 2812, 2614, 2423, 2288, 2245, 2286, 2375, 2486, 2614, 2744, 2835, 2872, 2897, 2989, 3224, 3604, 4053, 4480, 4831, 5069, 5175, 5184, 5153, 5106, 5024, 4897, 4731, 4534, 4322, },
	/* LAT: -10 */ {  3790, 3631, 3479, 3333, 3197, 3077, 2974, 2883, 2787, 2673, 2552, 2451, 2403, 2425, 2509, 2637, 2793, 2953, 3079, 3144, 3159, 3184, 3303, 3552, 3876, 4198, 4464, 4629, 4666, 4614, 4547, 4483, 4394, 4269, 4120, 3957, 3790, },
	/* LAT:   0 */ {  3412, 3320, 3237, 3165, 3109, 3072, 3046, 3028, 3006, 2958, 2880, 2785, 2703, 2669, 2708, 2809, 2942, 3078, 3194, 3270, 3302, 3324, 3396, 3551, 3758, 3971, 4152, 4260, 4267, 4200, 4112, 4020, 3908, 3776, 3643, 3519, 3412, },
	/* LAT:  10 */ {  3283, 3252, 3233, 3230, 3255, 3303, 3359, 3414, 3450, 3441, 3373, 3258, 3130, 3033, 3004, 3043, 3123, 3221, 3322, 3407, 3471, 3533, 3620, 3737, 3871, 4008, 4128, 4199, 4203, 4142, 4033, 3890, 3728, 3569, 3434, 3338, 3283, },
	/* LAT:  20 */ {  3400, 3404, 3431, 3486, 3578, 3700, 3830, 3948, 4030, 4043, 3970, 3828, 3660, 3518, 3440, 3425, 3459, 3531, 3627, 3724, 3814, 3912, 4022, 4133, 4241, 4351, 4453, 4519, 4531, 4474, 4338, 4136, 3910, 3701, 3538, 3438, 3400, },
	/* LAT:  30 */ {  3723, 3731, 3787, 3888, 4032, 4204, 4380, 4537, 4646, 4673, 4601, 4445, 4254, 4087, 3980, 3932, 3934, 3983, 4069, 4166, 4263, 4368, 4484, 4602, 4721, 4847, 4966, 5053, 5080, 5024, 4869, 4630, 4357, 4103, 3904, 3777, 3723, },
	/* LAT:  40 */ {  4222, 4222, 4289, 4414, 4583, 4771, 4955, 5112, 5217, 5243, 5174, 5025, 4836, 4660, 4530, 4453, 4425, 4446, 4505, 4584, 4669, 4766, 4880, 5013, 5164, 5325, 5476, 5586, 5627, 5574, 5420, 5183, 4909, 4649, 4439, 4295, 4222, },
	/* LAT:  50 */ {  4832, 4826, 4884, 4996, 5144, 5304, 5454, 5575, 5648, 5657, 5593, 5465, 5301, 5136, 4997, 4898, 4842, 4829, 4852, 4902, 4970, 5060, 5179, 5329, 5504, 5688, 5854, 5973, 6019, 5976, 5846, 5651, 5425, 5209, 5029, 4901, 4832, },
	/* LAT:  60 */ {  5392, 5382, 5412, 5477, 5564, 5658, 5745, 5810, 5843, 5833, 5778, 5683, 5561, 5432, 5312, 5215, 5148, 5113, 5110, 5137, 5193, 5279, 5396, 5543, 5709, 5878, 6026, 6131, 6176, 6155, 6073, 5946, 5799, 5656, 5533, 5443, 5392, },
	/* LAT:  70 */ {  5726, 5708, 5706, 5719, 5742, 5769, 5792, 5806, 5805, 5785, 5745, 5687, 5615, 5539, 5464, 5400, 5351, 5323, 5317, 5337, 5381, 5450, 5542, 5650, 5768, 5882, 5982, 6055, 6095, 6099, 6070, 6017, 5949, 5879, 5814, 5762, 5726, },
	/* LAT:  80 */ {  5789, 5772, 5758, 5747, 5737, 5728, 5718, 5706, 5690, 5672, 5649, 5623, 5596, 5568, 5543, 5522, 5508, 5503, 5507, 5522, 5548, 5583, 5626, 5674, 5723, 5772, 5814, 5849, 5873, 5887, 5889, 5882, 5868, 5850, 5829, 5808, 5789, },
	/* LAT:  90 */ {  5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, },
};