#pragma once

#include <cmath>
#include <stdint.h>
#include <time.h>

#include "Constants.h"
#include "pico/stdlib.h"
#include "pico/stdio.h"

// #define WHEEL_RADIUS   95   // Prototype
#define WHEEL_RADIUS  0 //107.5    // Tumbler
#define WHEEL_PERIMETER  2 * 3.14 * WHEEL_RADIUS
#define GEAR_RATIO 4.5

#define ENCODER_COUNT_PER_REV 600 * 4 * 4.5


#define RADIANS_PER_COUNT   2 * 3.14 / ENCODER_COUNT_PER_REV

// #define NUMERATOR   WHEEL_PERIMETER
#define DISTANCE_PER_COUNT  WHEEL_PERIMETER * RADIANS_PER_COUNT

//MAX RPM ; RPM = ( (V/r) / (2*pi) ) * 60
#define MAX_RPM_CLAMP 2000

//MM_PER_SEC
#define MAX_VEL_CLAMP 5.0

//Radians
#define MAX_POS_CLAMP 3.14

class DiffDriveState{
    private:
        // Quadrature decoder state
        volatile uint8_t left_prev_  = 0; // bits: A=bit0, B=bit1
        volatile uint8_t right_prev_ = 0;
        volatile int8_t  left_dir_last_  = 0;
        volatile int8_t  right_dir_last_ = 0;

        //Count with direction for postion computation
        int64_t LeftEncoderDirectionalTickCounter = 0;
        int64_t RightEncoderDirectionalTickCounter = 0;

        //Always counts up for velcoity computation
        int64_t LeftEncoderTickCounter = 0;
        int64_t RightEncoderTickCounter = 0;

        //Current state-------------------------
        int8_t LeftWheelDirection = 1;
        int8_t RightWheelDirection = 1;

        double LeftWheelPosition = 0;
        double RightWheelPosition = 0;

        double LeftWheelVelocity = 0;
        double RightWheelVelocity = 0;

        uint32_t LeftWheelRPM = 0;
        uint32_t RightWheelRPM = 0;
        //---------------------------------------


        //Used to check if the wheel has turned------
        int64_t LastLeftEncoderTickCounter = 0;
        int64_t LastRightEncoderTickCounter = 0;
        clock_t LastStateUpdateTime;
        //-------------------------------------------

        

        static DiffDriveState* instancePtr;

        clock_t clock(){
            return (clock_t) time_us_64() / 10000;}

        DiffDriveState(){
            LastStateUpdateTime = clock();
        }
        


    public:

        // void InitEncoders();

        DiffDriveState(const DiffDriveState& obj)= delete;

        static DiffDriveState* getInstance()
        {   
            /**
             * @brief Construct a new if object using singleton pattern
             * 
             */
            if (instancePtr == NULL)
            {
                instancePtr = new DiffDriveState();
                return instancePtr;
            }
            else
            {
                return instancePtr;
            }
        }

       
        //Getters and setters------------------------------------------------------
        int64_t GetLeftEncoderCount() { return LeftEncoderTickCounter;  }
        int64_t GetRightEncoderCount(){ return RightEncoderTickCounter; }

        void SetLeftWheelDirection(int8_t dir ) { LeftWheelDirection = dir; }
        void SetRightWheelDirection(int8_t dir ) { RightWheelDirection = dir; }
 
        float GetLeftWheelPosition() { return LeftWheelPosition;  }
        float GetRightWheelPosition(){ return RightWheelPosition; }
 
        float GetLeftWheelVelocity() { return LeftWheelVelocity;  }
        float GetRightWheelVelocity(){ return RightWheelVelocity; }
 
        uint32_t GetLeftWheelRPM() { return LeftWheelRPM;  }
        uint32_t GetRightWheelRPM(){ return RightWheelRPM; }
 
        void SetLeftWheelStopped() { LeftWheelVelocity=0;  LeftWheelRPM = 0;}
        void SetRightWheelStopped(){ RightWheelVelocity=0; RightWheelRPM = 0;}

        // Direction getters (return -1, 0, +1)
        int GetLeftDir()  const { return LeftWheelDirection; }
        int GetRightDir() const { return RightWheelDirection; }

        
        int64_t GetLeftEncoderSigned()  const { return LeftEncoderDirectionalTickCounter; }
        int64_t GetRightEncoderSigned() const { return RightEncoderDirectionalTickCounter; }
        //---------------------------------------------------------------------------


        /**
         * @brief Encoder Interrupt handle
         * 
         */

        void InitEncoders(){
            left_prev_  = (gpio_get(LEFT_ENCODER_PIN_1)?1:0) | ((gpio_get(LEFT_ENCODER_PIN_2)?1:0) << 1);
            right_prev_ = (gpio_get(RIGHT_ENCODER_PIN_1)?1:0) | ((gpio_get(RIGHT_ENCODER_PIN_2)?1:0) << 1);
        }

        // void IRQ_ENCODER(uint gpio, uint32_t events) {  
            
        //     switch(gpio){
        //         case LEFT_ENCODER_PIN_1:
        //             UpdateLeftEncoderCount();
        //             break;

        //         case RIGHT_ENCODER_PIN_1:
        //             UpdateRightEncoderCount();
        //             break;
        //     };
        // }

        void IRQ_ENCODER(uint /*gpio*/, uint32_t /*events*/) {
            // x4 transition table: idx = (prev<<2) | curr, returns -1/0/+1
            static const int8_t LUT[16] = {
                0, +1, -1,  0,
               -1,  0,  0, +1,
               +1,  0,  0, -1,
                0, -1, +1,  0
            };
            auto read_ab = [](uint a_pin, uint b_pin)->uint8_t{
                uint8_t a = gpio_get(a_pin) ? 1 : 0;
                uint8_t b = gpio_get(b_pin) ? 1 : 0;
                return (a) | (b << 1);
            };
            // LEFT
            uint8_t lc = read_ab(LEFT_ENCODER_PIN_1, LEFT_ENCODER_PIN_2);
            int8_t  ls = LUT[(left_prev_ << 2) | lc];
            if (ls) {
                LeftEncoderDirectionalTickCounter += ls;
                LeftEncoderTickCounter += (ls > 0 ? ls : -ls);
                LeftWheelDirection = (ls > 0) ? +1 : -1;
                left_dir_last_ = LeftWheelDirection;
                left_prev_ = lc;
            }
            // RIGHT
            uint8_t rc = read_ab(RIGHT_ENCODER_PIN_1, RIGHT_ENCODER_PIN_2);
            int8_t  rs = LUT[(right_prev_ << 2) | rc];
            if (rs) {
                RightEncoderDirectionalTickCounter += rs;
                RightEncoderTickCounter += (rs > 0 ? rs : -rs);
                RightWheelDirection = (rs > 0) ? +1 : -1;
                right_dir_last_ = RightWheelDirection;
                right_prev_ = rc;
            }
        }

        void UpdateLeftEncoderCount() {
            LeftEncoderDirectionalTickCounter += LeftWheelDirection*1;
            LeftEncoderTickCounter += 1;
        }


        void UpdateRightEncoderCount(){
            RightEncoderDirectionalTickCounter += RightWheelDirection*1; 
            RightEncoderTickCounter += 1;
        }
        //---------------------------------------------------------------------------



        /**
         * @brief Computes Wheel position
         * 
         */
        void updateWheelPosition(){

            //Update Left wheel position in Radians

            
            LeftWheelPosition = (LeftEncoderDirectionalTickCounter % ENCODER_COUNT_PER_REV) * RADIANS_PER_COUNT;
            LeftWheelPosition = std::ceil(LeftWheelPosition * 100.0) / 100.0;
            if(LeftWheelPosition < 0) LeftWheelPosition = 6.28 + LeftWheelPosition;


            //Update Right wheel position and velocity
            RightWheelPosition = (RightEncoderDirectionalTickCounter % ENCODER_COUNT_PER_REV) * RADIANS_PER_COUNT;
            RightWheelPosition = std::ceil(RightWheelPosition * 100.0) / 100.0;
            if(RightWheelPosition < 0) RightWheelPosition = 6.28  + RightWheelPosition;

        }
        //---------------------------------------------------------------------------

 
        /**
         * @brief Computes Wheel Velocity and RPM
         * 
         */
        void updateWheelState(){
            
            clock_t nowTime = clock();
            double executionTime = ((double)(nowTime - LastStateUpdateTime) / CLOCKS_PER_SEC)+0.00001;
            
            if(LastLeftEncoderTickCounter != LeftEncoderTickCounter){
                double left_ticks_per_second = std::abs(LastLeftEncoderTickCounter - LeftEncoderTickCounter)/executionTime;
                double left_rev_per_sec  = left_ticks_per_second/ENCODER_COUNT_PER_REV;
                double temp_left_rpm  = double(left_rev_per_sec * 60); // double(7.2);
                LeftWheelRPM = (unsigned int)(temp_left_rpm > MAX_RPM_CLAMP ? MAX_RPM_CLAMP : temp_left_rpm);

                double tempLeftWheelVelocity = left_ticks_per_second*DISTANCE_PER_COUNT;
                LeftWheelVelocity = tempLeftWheelVelocity > MAX_VEL_CLAMP ? MAX_VEL_CLAMP : tempLeftWheelVelocity;
            }
            else{
                SetLeftWheelStopped();
            }

            if(LastRightEncoderTickCounter != RightEncoderTickCounter){
                double right_ticks_per_second = std::abs(LastRightEncoderTickCounter - RightEncoderTickCounter)/executionTime;
                double right_rev_per_sec  = right_ticks_per_second/ENCODER_COUNT_PER_REV;
                double temp_right_rpm  = double(right_rev_per_sec * 60); // double(7.2);
                RightWheelRPM = (unsigned int)(temp_right_rpm > MAX_RPM_CLAMP ? MAX_RPM_CLAMP : temp_right_rpm);

                double tempRightWheelVelocity = right_ticks_per_second*DISTANCE_PER_COUNT;
                RightWheelVelocity = tempRightWheelVelocity > MAX_VEL_CLAMP ? MAX_VEL_CLAMP : tempRightWheelVelocity;
            }
            else{
                SetRightWheelStopped();
            }
        
            LastStateUpdateTime = nowTime;

            LastLeftEncoderTickCounter = LeftEncoderTickCounter;
            LastRightEncoderTickCounter = RightEncoderTickCounter;

        }
        //---------------------------------------------------------------------------
};