#ifndef TRANSMISSION_H
#define TRANSMISSION_H

void ParseCamString(float &ball_distance, float &ball_angle, float &goal_angle, float &distance_pixels);
void SendDataSerial(float filtered_angle, float ball_distance, float ball_angle, float goal_angle, float distance_pixels);

#endif