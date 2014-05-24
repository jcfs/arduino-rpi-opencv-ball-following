/*
   Generic Robot with obstacle avoidance capabilities
   Copyright (C) 2014  Joao Salavisa (joao.salavisa@gmail.com)

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef L298N_H
#define L298N_H

#define TOP_SPEED           255
#define MOTOR_A_TOP_SPEED   150
#define MOTOR_B_TOP_SPEED   170

class L298n {
    private:
        int _enable_motor_A_pin;
        int _enable_motor_B_pin;
        int _motor_A1_pin;
        int _motor_A2_pin;
        int _motor_B1_pin;
        int _motor_B2_pin;

    public:
        L298n(int enable_motor_A_pin, int motor_A1_pin, int motor_A2_pin, int enable_motor_B_pin, int motor_B1_pin, int motor_B2_pin);

        void setMotorsSpeed(int value);

        void forwardMotors();
        void brakeMotors();
        
        void forwardMotorA();
        void forwardMotorB();

        void backwardMotorA();
        void backwardMotorB();

        void brakeMotorA();
        void brakeMotorB();  
        
        void setMotorASpeed(int value);
        void setMotorBSpeed(int value);

};




#endif /* L298N_H */
