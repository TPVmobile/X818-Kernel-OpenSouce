/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef _CUST_BATTERY_METER_TABLE_H
#define _CUST_BATTERY_METER_TABLE_H


/* ============================================================
// define
// ============================================================*/
#define BAT_NTC_10 1
#define BAT_NTC_47 0

#if (BAT_NTC_10 == 1)
#define RBAT_PULL_UP_R             24000
#endif

#if (BAT_NTC_47 == 1)
#define RBAT_PULL_UP_R             61900
#endif

#define RBAT_PULL_UP_VOLT          1800



/* ============================================================
// ENUM
// ============================================================*/

/* ============================================================
// structure
// ============================================================*/

/* ============================================================
// typedef
// ============================================================*/
typedef struct _BATTERY_PROFILE_STRUCT {
	signed int percentage;
	signed int voltage;
	} BATTERY_PROFILE_STRUCT, *BATTERY_PROFILE_STRUCT_P;

typedef struct _R_PROFILE_STRUCT {
	signed int resistance; /* Ohm*/
	signed int voltage;
} R_PROFILE_STRUCT, *R_PROFILE_STRUCT_P;

typedef enum {
	T1_0C,
	T2_25C,
	T3_50C
} PROFILE_TEMPERATURE;

/* ============================================================
// External Variables
// ============================================================*/

/* ============================================================
// External function
// ============================================================*/

/* ============================================================
// <DOD, Battery_Voltage> Table
// ============================================================*/
#if (BAT_NTC_10 == 1)
BATT_TEMPERATURE Batt_Temperature_Table[] = {
	{-30, 111300},
	{-25, 86560},
	{-20, 67790},
	{-15, 53460},
	{-10, 42450},
	{ -5, 33930},
	{  0, 27280},
	{  5, 22070},
	{ 10, 17960},
	{ 15, 14700},
	{ 20, 12090},
	{ 25, 10000},
	{ 30, 8312},
	{ 35, 6942},
	{ 40, 5826},
	{ 45, 4911},
	{ 50, 4158},
	{ 55, 3536},
	{ 60, 3019},
	{ 65, 2588},
	{ 70, 2227}
};
#endif

#if (BAT_NTC_47 == 1)
BATT_TEMPERATURE Batt_Temperature_Table[] = {
	{-20, 483954},
	{-15, 360850},
	{-10, 271697},
	{ -5, 206463},
	{  0, 158214},
	{  5, 122259},
	{ 10, 95227},
	{ 15, 74730},
	{ 20, 59065},
	{ 25, 47000},
	{ 30, 37643},
	{ 35, 30334},
	{ 40, 24591},
	{ 45, 20048},
	{ 50, 16433},
	{ 55, 13539},
	{ 60, 11210}
};
#endif

/* T0 -10C */
BATTERY_PROFILE_STRUCT battery_profile_t0[] = {
	{ 0,4359},
	{ 1,4340},
	{ 2,4322},
	{ 4,4306},
	{ 5,4290},
	{ 6,4276},
	{ 7,4261},
	{ 9,4246},
	{10,4232},
	{11,4217},
	{12,4203},
	{13,4189},
	{15,4175},
	{16,4162},
	{17,4148},
	{18,4134},
	{20,4120},
	{21,4106},
	{22,4094},
	{23,4082},
	{24,4070},
	{26,4056},
	{27,4037},
	{28,4016},
	{29,3996},
	{30,3979},
	{32,3965},
	{33,3954},
	{34,3945},
	{35,3936},
	{37,3928},
	{38,3919},
	{39,3911},
	{40,3902},
	{41,3893},
	{43,3885},
	{44,3876},
	{45,3869},
	{46,3860},
	{48,3854},
	{49,3847},
	{50,3841},
	{51,3834},
	{52,3828},
	{54,3823},
	{55,3817},
	{56,3812},
	{57,3807},
	{59,3802},
	{60,3798},
	{61,3794},
	{62,3792},
	{63,3789},
	{65,3786},
	{66,3784},
	{67,3781},
	{68,3779},
	{69,3776},
	{71,3773},
	{72,3770},
	{73,3767},
	{74,3764},
	{76,3760},
	{77,3756},
	{78,3752},
	{79,3747},
	{80,3741},
	{82,3736},
	{83,3729},
	{84,3723},
	{85,3717},
	{87,3711},
	{88,3706},
	{89,3701},
	{90,3695},
	{91,3687},
	{93,3676},
	{94,3655},
	{95,3621},
	{96,3566},
	{100,3400},
	{100,3400},
	{100,3400},
	{100,3400},
	{100,3400},
	{100,3400},
	{100,3400},
	{100,3400},
	{100,3400},
	{100,3400},
	{100,3400},
	{100,3400},
	{100,3400},
	{100,3400},
	{100,3400},
	{100,3400},
	{100,3400},
	{100,3400},
	{100,3400},
	{100,3400},
};

/* T1 0C */
BATTERY_PROFILE_STRUCT battery_profile_t1[] = {
	{  0,4343},
	{  1,4324},
	{  3,4307},
	{  4,4292},
	{  5,4276},
	{  6,4262},
	{  8,4247},
	{  9,4233},
	{ 10,4218},
	{ 11,4204},
	{ 13,4190},
	{ 14,4176},
	{ 15,4163},
	{ 16,4149},
	{ 18,4136},
	{ 19,4122},
	{ 20,4109},
	{ 21,4096},
	{ 23,4085},
	{ 24,4075},
	{ 25,4064},
	{ 26,4048},
	{ 28,4028},
	{ 29,4007},
	{ 30,3990},
	{ 31,3976},
	{ 33,3965},
	{ 34,3956},
	{ 35,3946},
	{ 36,3936},
	{ 38,3926},
	{ 39,3916},
	{ 40,3905},
	{ 41,3895},
	{ 43,3885},
	{ 44,3876},
	{ 45,3868},
	{ 47,3860},
	{ 48,3852},
	{ 49,3846},
	{ 50,3839},
	{ 52,3833},
	{ 53,3827},
	{ 54,3822},
	{ 55,3816},
	{ 57,3812},
	{ 58,3807},
	{ 59,3803},
	{ 60,3798},
	{ 62,3794},
	{ 63,3791},
	{ 64,3787},
	{ 65,3785},
	{ 67,3783},
	{ 68,3781},
	{ 69,3779},
	{ 70,3776},
	{ 72,3774},
	{ 73,3772},
	{ 74,3770},
	{ 75,3767},
	{ 77,3764},
	{ 78,3760},
	{ 79,3756},
	{ 80,3751},
	{ 82,3746},
	{ 83,3740},
	{ 84,3733},
	{ 85,3726},
	{ 87,3718},
	{ 88,3710},
	{ 89,3702},
	{ 90,3698},
	{ 92,3694},
	{ 93,3689},
	{ 94,3682},
	{ 96,3669},
	{ 97,3636},
	{ 98,3575},
	{ 99,3481},
	{100,3388},
	{100,3388},
	{100,3388},
	{100,3388},
	{100,3388},
	{100,3388},
	{100,3388},
	{100,3388},
	{100,3388},
	{100,3388},
	{100,3388},
	{100,3388},
	{100,3388},
	{100,3388},
	{100,3388},
	{100,3388},
	{100,3388},
	{100,3388},
	{100,3388},
	{100,3388},
};

/* T2 25C*/
BATTERY_PROFILE_STRUCT battery_profile_t2[] = {
	{  0,4387},
	{  1,4365},
	{  2,4348},
	{  4,4332},
	{  5,4317},
	{  6,4302},
	{  7,4287},
	{  9,4273},
	{ 10,4258},
	{ 11,4243},
	{ 12,4228},
	{ 14,4214},
	{ 15,4199},
	{ 16,4185},
	{ 17,4171},
	{ 18,4156},
	{ 20,4143},
	{ 21,4129},
	{ 22,4115},
	{ 23,4101},
	{ 25,4088},
	{ 26,4076},
	{ 27,4065},
	{ 28,4053},
	{ 29,4040},
	{ 31,4024},
	{ 32,4010},
	{ 33,3999},
	{ 34,3988},
	{ 36,3978},
	{ 37,3968},
	{ 38,3959},
	{ 39,3949},
	{ 41,3939},
	{ 42,3927},
	{ 43,3914},
	{ 44,3900},
	{ 45,3886},
	{ 47,3874},
	{ 48,3864},
	{ 49,3856},
	{ 50,3848},
	{ 52,3841},
	{ 53,3835},
	{ 54,3829},
	{ 55,3824},
	{ 57,3818},
	{ 58,3813},
	{ 59,3809},
	{ 60,3804},
	{ 61,3800},
	{ 63,3796},
	{ 64,3792},
	{ 65,3789},
	{ 66,3786},
	{ 68,3782},
	{ 69,3779},
	{ 70,3776},
	{ 71,3774},
	{ 72,3771},
	{ 74,3768},
	{ 75,3765},
	{ 76,3761},
	{ 77,3757},
	{ 79,3752},
	{ 80,3747},
	{ 81,3743},
	{ 82,3739},
	{ 84,3733},
	{ 85,3724},
	{ 86,3717},
	{ 87,3710},
	{ 88,3699},
	{ 90,3692},
	{ 91,3690},
	{ 92,3688},
	{ 93,3687},
	{ 95,3684},
	{ 96,3676},
	{ 97,3636},
	{ 98,3569},
	{100,3471},
	{101,3293},
	{101,3293},
	{101,3293},
	{101,3293},
	{101,3293},
	{101,3293},
	{101,3293},
	{101,3293},
	{101,3293},
	{101,3293},
	{101,3293},
	{101,3293},
	{101,3293},
	{101,3293},
	{101,3293},
	{101,3293},
	{101,3293},
	{101,3293},

};

/* T3 50C*/
BATTERY_PROFILE_STRUCT battery_profile_t3[] = {
	{  0,4352},
	{  1,4335},
	{  3,4318},
	{  4,4303},
	{  5,4288},
	{  6,4273},
	{  8,4258},
	{  9,4244},
	{ 10,4229},
	{ 11,4215},
	{ 13,4200},
	{ 14,4186},
	{ 15,4172},
	{ 16,4158},
	{ 18,4144},
	{ 19,4131},
	{ 20,4117},
	{ 21,4104},
	{ 23,4091},
	{ 24,4078},
	{ 25,4065},
	{ 26,4053},
	{ 28,4041},
	{ 29,4029},
	{ 30,4017},
	{ 31,4006},
	{ 33,3996},
	{ 34,3985},
	{ 35,3975},
	{ 36,3965},
	{ 38,3956},
	{ 39,3946},
	{ 40,3937},
	{ 41,3927},
	{ 43,3915},
	{ 44,3900},
	{ 45,3885},
	{ 47,3873},
	{ 48,3864},
	{ 49,3856},
	{ 50,3849},
	{ 52,3843},
	{ 53,3836},
	{ 54,3830},
	{ 55,3825},
	{ 57,3820},
	{ 58,3815},
	{ 59,3811},
	{ 60,3806},
	{ 62,3802},
	{ 63,3798},
	{ 64,3794},
	{ 65,3791},
	{ 67,3788},
	{ 68,3785},
	{ 69,3782},
	{ 70,3779},
	{ 72,3773},
	{ 73,3764},
	{ 74,3756},
	{ 75,3751},
	{ 77,3747},
	{ 78,3742},
	{ 79,3737},
	{ 80,3732},
	{ 82,3728},
	{ 83,3723},
	{ 84,3716},
	{ 85,3708},
	{ 87,3701},
	{ 88,3691},
	{ 89,3683},
	{ 91,3682},
	{ 92,3680},
	{ 93,3679},
	{ 94,3677},
	{ 96,3668},
	{ 97,3630},
	{ 98,3568},
	{ 99,3479},
	{101,3334},
	{101,3334},
	{101,3334},
	{101,3334},
	{101,3334},
	{101,3334},
	{101,3334},
	{101,3334},
	{101,3334},
	{101,3334},
	{101,3334},
	{101,3334},
	{101,3334},
	{101,3334},
	{101,3334},
	{101,3334},
	{101,3334},
	{101,3334},
	{101,3334},
	{101,3334},

};

/* battery profile for actual temperature. The size should be the same as T1, T2 and T3*/
BATTERY_PROFILE_STRUCT battery_profile_temperature[] = {
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0}
};

/* ============================================================
// <Rbat, Battery_Voltage> Table
// ============================================================*/
/* T0 -10C*/
R_PROFILE_STRUCT r_profile_t0[] = {
	{ 193,4359},
	{ 817,4340},
	{ 788,4322},
	{ 781,4306},
	{ 776,4290},
	{ 768,4276},
	{ 761,4261},
	{ 755,4246},
	{ 745,4232},
	{ 738,4217},
	{ 730,4203},
	{ 723,4189},
	{ 717,4175},
	{ 711,4162},
	{ 704,4148},
	{ 695,4134},
	{ 688,4120},
	{ 683,4106},
	{ 680,4094},
	{ 679,4082},
	{ 677,4070},
	{ 669,4056},
	{ 654,4037},
	{ 634,4016},
	{ 618,3996},
	{ 609,3979},
	{ 607,3965},
	{ 609,3954},
	{ 611,3945},
	{ 612,3936},
	{ 610,3928},
	{ 607,3919},
	{ 605,3911},
	{ 602,3902},
	{ 599,3893},
	{ 599,3885},
	{ 598,3876},
	{ 601,3869},
	{ 601,3860},
	{ 604,3854},
	{ 606,3847},
	{ 609,3841},
	{ 611,3834},
	{ 614,3828},
	{ 618,3823},
	{ 621,3817},
	{ 625,3812},
	{ 629,3807},
	{ 633,3802},
	{ 639,3798},
	{ 643,3794},
	{ 654,3792},
	{ 665,3789},
	{ 675,3786},
	{ 686,3784},
	{ 695,3781},
	{ 706,3779},
	{ 718,3776},
	{ 729,3773},
	{ 741,3770},
	{ 755,3767},
	{ 768,3764},
	{ 784,3760},
	{ 801,3756},
	{ 819,3752},
	{ 838,3747},
	{ 858,3741},
	{ 883,3736},
	{ 909,3729},
	{ 940,3723},
	{ 977,3717},
	{1024,3711},
	{1082,3706},
	{1153,3701},
	{1241,3695},
	{1355,3687},
	{1503,3676},
	{1650,3655},
	{1766,3621},
	{1869,3566},
	{3000,3400},
	{3000,3400},
	{3000,3400},
	{3000,3400},
	{3000,3400},
	{3000,3400},
	{3000,3400},
	{3000,3400},
	{3000,3400},
	{3000,3400},
	{3000,3400},
	{3000,3400},
	{3000,3400},
	{3000,3400},
	{3000,3400},
	{3000,3400},
	{3000,3400},
	{3000,3400},
	{3000,3400},
	{3000,3400},
};

/* T1 0C*/
R_PROFILE_STRUCT r_profile_t1[] = {
	{ 180,4343},
	{ 435,4324},
	{ 432,4307},
	{ 428,4292},
	{ 426,4276},
	{ 422,4262},
	{ 419,4247},
	{ 416,4233},
	{ 414,4218},
	{ 412,4204},
	{ 411,4190},
	{ 409,4176},
	{ 410,4163},
	{ 408,4149},
	{ 409,4136},
	{ 408,4122},
	{ 408,4109},
	{ 407,4096},
	{ 411,4085},
	{ 419,4075},
	{ 426,4064},
	{ 423,4048},
	{ 412,4028},
	{ 402,4007},
	{ 395,3990},
	{ 392,3976},
	{ 390,3965},
	{ 388,3956},
	{ 385,3946},
	{ 377,3936},
	{ 372,3926},
	{ 370,3916},
	{ 366,3905},
	{ 363,3895},
	{ 359,3885},
	{ 358,3876},
	{ 356,3868},
	{ 357,3860},
	{ 357,3852},
	{ 358,3846},
	{ 357,3839},
	{ 361,3833},
	{ 361,3827},
	{ 363,3822},
	{ 364,3816},
	{ 365,3812},
	{ 369,3807},
	{ 370,3803},
	{ 373,3798},
	{ 375,3794},
	{ 377,3791},
	{ 380,3787},
	{ 385,3785},
	{ 390,3783},
	{ 396,3781},
	{ 402,3779},
	{ 409,3776},
	{ 415,3774},
	{ 423,3772},
	{ 431,3770},
	{ 439,3767},
	{ 447,3764},
	{ 457,3760},
	{ 467,3756},
	{ 480,3751},
	{ 493,3746},
	{ 506,3740},
	{ 522,3733},
	{ 540,3726},
	{ 558,3718},
	{ 581,3710},
	{ 609,3702},
	{ 647,3698},
	{ 696,3694},
	{ 760,3689},
	{ 845,3682},
	{ 891,3669},
	{1126,3636},
	{1399,3575},
	{1837,3481},
	{2242,3388},
	{2242,3388},
	{2242,3388},
	{2242,3388},
	{2242,3388},
	{2242,3388},
	{2242,3388},
	{2242,3388},
	{2242,3388},
	{2242,3388},
	{2242,3388},
	{2242,3388},
	{2242,3388},
	{2242,3388},
	{2242,3388},
	{2242,3388},
	{2242,3388},
	{2242,3388},
	{2242,3388},
	{2242,3388},
};

/* T2 25C*/
R_PROFILE_STRUCT r_profile_t2[] = {
	{128,4387},
	{160,4365},
	{161,4348},
	{160,4332},
	{158,4317},
	{158,4302},
	{157,4287},
	{159,4273},
	{158,4258},
	{157,4243},
	{158,4228},
	{159,4214},
	{160,4199},
	{160,4185},
	{161,4171},
	{161,4156},
	{163,4143},
	{164,4129},
	{165,4115},
	{166,4101},
	{168,4088},
	{169,4076},
	{173,4065},
	{177,4053},
	{180,4040},
	{176,4024},
	{177,4010},
	{178,3999},
	{176,3988},
	{178,3978},
	{178,3968},
	{181,3959},
	{181,3949},
	{182,3939},
	{178,3927},
	{172,3914},
	{163,3900},
	{154,3886},
	{149,3874},
	{146,3864},
	{144,3856},
	{143,3848},
	{143,3841},
	{143,3835},
	{144,3829},
	{145,3824},
	{145,3818},
	{147,3813},
	{148,3809},
	{150,3804},
	{149,3800},
	{151,3796},
	{151,3792},
	{153,3789},
	{155,3786},
	{154,3782},
	{154,3779},
	{154,3776},
	{155,3774},
	{154,3771},
	{154,3768},
	{154,3765},
	{150,3761},
	{149,3757},
	{147,3752},
	{147,3747},
	{148,3743},
	{147,3739},
	{148,3733},
	{147,3724},
	{147,3717},
	{148,3710},
	{147,3699},
	{146,3692},
	{148,3690},
	{153,3688},
	{159,3687},
	{167,3684},
	{179,3676},
	{172,3636},
	{184,3569},
	{208,3471},
	{286,3293},
	{286,3293},
	{286,3293},
	{286,3293},
	{286,3293},
	{286,3293},
	{286,3293},
	{286,3293},
	{286,3293},
	{286,3293},
	{286,3293},
	{286,3293},
	{286,3293},
	{286,3293},
	{286,3293},
	{286,3293},
	{286,3293},
	{286,3293},

};

/* T3 50C*/
R_PROFILE_STRUCT r_profile_t3[] = {
	{115,4352},
	{118,4335},
	{117,4318},
	{117,4303},
	{117,4288},
	{117,4273},
	{117,4258},
	{118,4244},
	{117,4229},
	{118,4215},
	{118,4200},
	{118,4186},
	{118,4172},
	{120,4158},
	{119,4144},
	{120,4131},
	{121,4117},
	{120,4104},
	{122,4091},
	{123,4078},
	{122,4065},
	{124,4053},
	{124,4041},
	{125,4029},
	{126,4017},
	{126,4006},
	{128,3996},
	{129,3985},
	{131,3975},
	{133,3965},
	{136,3956},
	{138,3946},
	{142,3937},
	{144,3927},
	{143,3915},
	{135,3900},
	{126,3885},
	{120,3873},
	{119,3864},
	{118,3856},
	{118,3849},
	{119,3843},
	{117,3836},
	{118,3830},
	{118,3825},
	{119,3820},
	{119,3815},
	{119,3811},
	{120,3806},
	{122,3802},
	{121,3798},
	{123,3794},
	{123,3791},
	{124,3788},
	{126,3785},
	{127,3782},
	{128,3779},
	{125,3773},
	{118,3764},
	{117,3756},
	{119,3751},
	{119,3747},
	{119,3742},
	{118,3737},
	{119,3732},
	{119,3728},
	{119,3723},
	{120,3716},
	{119,3708},
	{120,3701},
	{119,3691},
	{116,3683},
	{119,3682},
	{120,3680},
	{124,3679},
	{130,3677},
	{133,3668},
	{126,3630},
	{133,3568},
	{142,3479},
	{175,3334},
	{175,3334},
	{175,3334},
	{175,3334},
	{175,3334},
	{175,3334},
	{175,3334},
	{175,3334},
	{175,3334},
	{175,3334},
	{175,3334},
	{175,3334},
	{175,3334},
	{175,3334},
	{175,3334},
	{175,3334},
	{175,3334},
	{175,3334},
	{175,3334},
	{175,3334},

};

/* r-table profile for actual temperature. The size should be the same as T1, T2 and T3*/
R_PROFILE_STRUCT r_profile_temperature[] = {
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0}
};

/* ============================================================
// function prototype
// ============================================================*/
int fgauge_get_saddles(void);
BATTERY_PROFILE_STRUCT_P fgauge_get_profile(unsigned int temperature);

int fgauge_get_saddles_r_table(void);
R_PROFILE_STRUCT_P fgauge_get_profile_r_table(unsigned int temperature);

#endif
