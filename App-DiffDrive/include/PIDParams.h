/**
 * @brief PID CONTEXT ------------------------------------------------------------------
 */
// PID Coefficients
double _kP_l = 0.1;
double _kI_l = 0.001;
double _kD_l = 0.0001;

double _kP_r = 0.1;
double _kI_r = 0.001;
double _kD_r = 0.0001;//0.000000000625;

// PID Data
double rITerm = 0.0;
double lITerm = 0.0;

// Current Speed
int rLastSpeed = 0;
int lLastSpeed = 0;

// Temp adjusted PWN
double l_pwm = 0.0;
double r_pwm = 0.0;

// Current PWM
uint8_t LEFT_PWM = 0;
uint8_t RIGHT_PWM = 0;
//--------------------------------------------------------------------------------------
