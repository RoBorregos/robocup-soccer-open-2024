#include <iostream>
#include <Wire.h>
#include "Bno.h"
#include <cmath>
#include <Pixy2SPI_SS.h>

/*
Pixy camera resolution: 316 x 208
pixy.ccc.blocks[i].m_x The x location of the center of the detected object (0 to 316)
pixy.ccc.blocks[i].m_y The y location of the center of the detected object (0 to 208)
*/

/*
Pixy cam con la que veo de default puedo ver la distancia y el angulo de todas mis cosas, hay un rango en donde ya no vi mi pelota. Entonces se debe
de tener tres condiciones de cuando no veo la pelota checar si la puedo ver en la openmv, de cuando la veo en las dos y de cuando solo la veo en la pixy. En
base a eso puedo acercarme a la pelota en su ball angle y cuando ya la tenga enfrente (osea que la vea con la camara frontal openmv a cierta distancia y angulo)
 y si esta ahi prendo dribbler y avanzo hacia el angulo de la pelota de acuerdo a mi openmv en caso de no ver la pelota 
en ningun lado, me dirigo al angulo donde la vi por ultima vez
*/

float bno_angle = 0;
float distance_pixels = 0;
float distance_pixels_protect = 0;
float ball_angle = 0;
float ball_distance = 0;
float goal_angle = 0;
float goal_distance = 0;
double last_time = 0;
double current_time = 0;
float angle = 0;
const int FRAME_HEIGHT = 104;
const int FRAME_WIDTH = 158;

PID pid_w(0.6, 0.01, 6, 200);
Pixy2SPI_SS pixy;
BNO055 my_bno;

void setup()
{
  Serial1.begin(115200);
  Serial.begin(9500);
  pixy.init();
  my_bno.InitializeBNO();
}

double radiansToDegrees(double radians)
{
  return radians * (180.0 / M_PI);
}

void loop()
{

  // ----------------- Gather data from OpenMV camera via UART ----------------- //
  my_bno.GetBNOData();
  angle = my_bno.GetYaw();
  /*if (Serial1.available())
  {
    String camString = Serial1.readStringUntil('\n');
    ball_distance = camString.toFloat();
    ball_angle = camString.substring(camString.indexOf(' ') + 1).toFloat();
    goal_angle = camString.substring(camString.indexOf(' ', camString.indexOf(' ') + 1) + 1, camString.lastIndexOf(' ')).toFloat();
    distance_pixels = camString.substring(camString.lastIndexOf(' ') + 1).toFloat();
    distance_pixels_protect = camString.substring(camString.indexOf(' ', camString.lastIndexOf(' ') + 1)).toFloat();
  }
  String angleString = String(angle);
  String ballDistance = String(ball_distance);
  String ballAngle = String(ball_angle);*/

  // ----------------- Gather data from Pixy2 camera via SPI ----------------- //

  int i;
  pixy.ccc.getBlocks();
  int pixy_blocks = pixy.ccc.numBlocks;
  bool ball_seen_pixy = false;
  //bool ball_seen_openmv = (ball_distance != 0 && ball_angle != 0);

  if (pixy.ccc.numBlocks)
  {
    for (i = 0; i < pixy.ccc.numBlocks; i++)
    {
      double cx = pixy.ccc.blocks[i].m_x;
      double cy = pixy.ccc.blocks[i].m_y; 
      double relative_cx = cx - FRAME_WIDTH;
      double relative_cy = cy - FRAME_HEIGHT;
      double magnitude_distance = sqrt(relative_cx * relative_cx + relative_cy * relative_cy);
      double total_distance = 1.2415 * magnitude_distance - 5.2805;


      double angle_radians = atan2(relative_cy, relative_cx);
      double angle_degrees = radiansToDegrees(angle_radians);
 
      angle_degrees += 8;
      if (angle_degrees < 0)
      {
        angle_degrees += 360;
      }
      if (angle_degrees >= 360)
      {
        angle_degrees -= 360;
      }

      // A certain signature means the ball, update values for both cams, because the ball is the same in both views just different POVs
      if(pixy.ccc.blocks[i].m_signature == 1)
      {
        ball_seen_pixy = true;
        ball_distance = total_distance; 
        ball_angle = angle_degrees;
      }

    }
  }

  double speed_w = pid_w.Calculate(target_angle, bno_angle);
  double speed_t_goal = 150;
  double speed_t_ball = 150;

  if (speed_w != 0)
        {
            //--------------------Separate coordinate plane--------------------------//

            if (ball_angle < 180)
            {
                ball_angle_180 = -ball_angle;
            }
            else if (ball_angle > 180)
            {
                ball_angle_180 = 360 - ball_angle;
            }

            //---------------------------Logic for ball found-------------------------------//

            if (ball_angle == 0)
            {
                ball_found = false;
            }
            else
            {
                ball_found = true;
            }
            if (ball_found)
            {
                Serial.println("BALL FOUND");
                if (ball_angle_180 > -15 && ball_angle_180 < 15)
                {

                    myMotors.MoveMotorsImu(0, abs(speed_t_ball), speed_w);
                }
                else
                {
                    Serial.println("PELOTA ANGLE");
                    ball_angle = 360 - ball_angle;
                    double differential = ball_angle * 0.12;
                    ponderated_angle = ball_angle - differential;
                    ponderated_angle = ball_angle > 180 ? ball_angle - differential : ball_angle + differential;
                    myMotors.MoveMotorsImu(ponderated_angle, abs(speed_t_ball), speed_w);
                }
            }
        }

  /*if(ball_seen_pixy && ball_seen_openmv)
  {
    Serial.println("Both cameras see the ball");
  }
  else if(ball_seen_pixy && !ball_seen_openmv)
  {
    Serial.println("Only pixy sees the ball");
  }
  else if(!ball_seen_pixy && ball_seen_openmv)
  {
    Serial.println("Only openmv sees the ball");
  }
  else
  {
    Serial.println("No camera sees the ball, move to last known position");
  }*/
}
