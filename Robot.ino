#include "Logger.h"
#include "SonarHead.h"
#include "MotorDriver.h"

#define ENA 3  //Right PWM
#define IN1 4  //Right FWD
#define IN2 5  //Right BWD

#define ENB 6  //Left PWM
#define IN3 7  //Left FWD
#define IN4 8  //Left BWD

#define SERVO_PIN 9
#define SONAR_TRIG_PIN 12
#define SONAR_ECHO_PIN 13

SonarHead m_sonar_head(SONAR_TRIG_PIN, SONAR_ECHO_PIN, SERVO_PIN);
MotorDriver m_motor(ENA, IN1, IN2, ENB, IN3, IN4);

// ----------------------------------------------
enum State
{
    ANALYZE,
    TURN,
    DRIVE,
    REVERSE,
};
State m_state;

enum Direction
{
    LEFT    = 0,
    FORWARD = 1,
    RIGHT   = 2,
};

float m_distance[] = {0, 0, 0};
float m_safe_distance = 1.f;
float m_stop_distance = 0.5f;

#define TEST_MOTOR

// ----------------------------------------------
void setup()
{
    m_motor.set_turn_pwm(150);
    m_motor.set_drive_pwm(200);

#ifdef TEST_MOTOR
    // test motors  
    Logger::instance().log(0, 0, "Testing Drives");
    m_motor.test_drives(100, 500, 250);
    Logger::instance().clear();
#endif 

    set_state(ANALYZE);
}

// ----------------------------------------------

void print(uint8_t col, uint8_t row, const char* text)
{
    Logger::instance().log(col, row, text);
}

void print_distance(float distance, const char* postfix)
{
    Logger::instance().log(distance, postfix);
}

// ----------------------------------------------
void set_state(State new_state)
{
    m_state = new_state;
    
    switch(m_state)
    {
    case ANALYZE:   print(0, 0, "Analyzing");   break;
    case TURN:      print(0, 0, "Turning  ");   break;
    case DRIVE:     print(0, 0, "Driving  ");   break;
    case REVERSE:   print(0, 0, "Reversing");   break;
    }
}

void analyze(int delay_ms)
{
    // turn head, store distance
    scan(RIGHT,   delay_ms);
    scan(FORWARD, delay_ms);
    scan(LEFT,    delay_ms);
    
    // turn head back
    scan(FORWARD, 500);

    // analyze distances and choose direction
    if (m_distance[FORWARD] > m_safe_distance)
        set_state(DRIVE);
    else if (m_distance[RIGHT] > m_safe_distance || m_distance[LEFT] > m_safe_distance)
        set_state(TURN);
    else
        set_state(REVERSE);
}

void scan(Direction direction, int delay_ms)
{
    switch(direction)
    {
    case LEFT:
        m_distance[direction] = m_sonar_head.scan_direction(SonarHead::Right, delay_ms);
        print(0, 1, "Right: ");
        break;
    case FORWARD:
        m_distance[direction] = m_sonar_head.scan_direction(SonarHead::Forward, delay_ms);
        print(0, 1, "Fwd  : ");
        break;
    case RIGHT:
        m_distance[direction] = m_sonar_head.scan_direction(SonarHead::Left, delay_ms);
        print(0, 1, "Left : ");
        break;
    }
    
    print_distance(m_distance[direction], " cm   ");
    delay(delay_ms);
}

void turn(int duration)
{
    if (m_distance[RIGHT] > m_distance[LEFT])
    {
        print(0, 1, "Right            ");
        m_motor.turn_right();
    }
    else
    {
        print(0, 1, "Left            ");
        m_motor.turn_left();
    }
    
    delay(duration);
    m_motor.stop();

    set_state(ANALYZE);
}

void drive()
{
    float distance = m_sonar_head.scan_direction(0, 0);
    
    String log_string("Dist: ");        // 6
    log_string += String(distance, 2);  // 4
    log_string += " cm ";               // 4

    print(0, 1, "Dist : ");
    print_distance(distance, " cm   ");

    if (distance > m_safe_distance)
    {
        print(11, 1, "Fast");
        log_string += "Fast";
        m_motor.forward();
    }
    else if (distance > m_stop_distance)
    {
        print(11, 1, "Slow");
        log_string += "Slow";

        //*
        float speed_factor = map(distance, m_stop_distance, m_safe_distance, 0, 100) * 0.01;
        m_motor.forward(0.25f + (speed_factor * 0.75f));
        /*/
        m_motor.forward(100);
        //*/
    }    
    else
    {
        print(11, 1, "Stop");
        log_string += "Stop";

        m_motor.stop();
        set_state(ANALYZE);
    }

    Logger::instance().log(log_string);
}

void reverse(int duration)
{
    m_motor.backward();
    delay(duration);
    m_motor.stop();

    // re-evaluate
    set_state(ANALYZE);
}

// ----------------------------------------------
void loop()
{
    switch(m_state)
    {
    case ANALYZE:   analyze(1000);  break;
    case TURN:      turn(500);      break;
    case DRIVE:     drive();        break;
    case REVERSE:   reverse(2000);  break;
    }
}