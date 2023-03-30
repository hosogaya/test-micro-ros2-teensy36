/**
 * @file PINs.h
 * @author Hirotaka Hosogaya (hosogaya.hirotaka@b.mbox.nagoaya-u.ac.jp)
 * @brief In this file, number of each pins are defined by whose role.
 * @version 0.1
 * @date 2021-12-04
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#pragma once
#include <Arduino.h>

#define CONVERTER_SWITCH 5
#define RELAY_VOLTAGE 6
#define RELAY_SWITCH 30

#define GPIO1 26
#define GPIO2 27
#define GPIO3 28
#define GPIO4 29

#define CN20 23
#define CN19 22
#define CN18 21
#define CN17 20
#define CN16 25
#define CN15 24
#define CN14 16
#define CN13 17

#define ORANGE_LED 13

#define SCL0 19
#define SDA0 18

#define SCL1 37
#define SDA1 38

#define SCL2 3
#define SDA2 4


void setupPins() {
	pinMode(CONVERTER_SWITCH, OUTPUT); // Switch of level converter for m-> (3.3V <-> 5V) High:ON, Low: OFF
    pinMode(RELAY_VOLTAGE, INPUT_PULLUP); // voltage of relay switch (High:ON, Low;OFF) 
    pinMode(RELAY_SWITCH, OUTPUT); // Switch of relay (Low: ON, High: OFF)
    pinMode(ORANGE_LED, OUTPUT);
    digitalWrite(CONVERTER_SWITCH, HIGH); delay(100);
    digitalWrite(RELAY_SWITCH, LOW); delay(100);
    digitalWrite(ORANGE_LED, HIGH);	
}