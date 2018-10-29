#include "Logger.h"
#include "SonarHead.h"
#include "MotorDriver.h"

#define ENA 3  //Right PWM
#define IN1 4  //Right FWD
#define IN2 5  //Right BWD

#define ENB 6  //Left PWM
#define IN3 7  //Left FWD
#define IN4 8  //Left BWD

#define SERVO_PIN      9
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

float m_safe_distance = 1.f;
float m_stop_distance = 0.5f;

// #define TEST_CYCLE

// ----------------------------------------------
void setup()
{
    m_motor.init();
    m_motor.set_turn_pwm(150);
    m_motor.set_drive_pwm(200);

    m_sonar_head.init();
    m_sonar_head.set_scan_delay(250);

#ifdef TEST_CYCLE
    m_sonar_head.test_servo(1000);              // test servo
    m_motor.test_drives(100, 500, 250);         // test motors  
#endif // TEST_CYCLE

    set_state(ANALYZE);
}

// ----------------------------------------------
void set_state(State new_state)
{
    m_state = new_state;
    
    switch(m_state)
    {
    case ANALYZE:   Logger::instance().log(0, 0, "Analyzing       ");   break;
    case TURN:      Logger::instance().log(0, 0, "Turning         ");   break;
    case DRIVE:     Logger::instance().log(0, 0, "Driving         ");   break;
    case REVERSE:   Logger::instance().log(0, 0, "Reversing       ");   break;
    }
}

void analyze(uint16_t delay_ms)
{
    // sweep-scan forward hemisphere
    m_sonar_head.sweep(SonarHead::Right, SonarHead::Left, 5);
    
    // turn head back
    m_sonar_head.look_at(SonarHead::Forward);

    m_sonar_head.log_distances(1000);

    /*
    Logger::instance().log(0, 0, String(m_sonar_head.scanned_distance(SonarHead::Left), 2) + " " +
                                 String(m_sonar_head.scanned_distance(SonarHead::Forward), 2) + " " +
                                 String(m_sonar_head.scanned_distance(SonarHead::Right), 2));
    delay(2000);
    */

    // analyze distances and choose direction
    if (m_sonar_head.scanned_distance(SonarHead::Forward) > m_safe_distance)
        set_state(DRIVE);
    else if (m_sonar_head.scanned_distance(SonarHead::Right) > m_safe_distance ||
             m_sonar_head.scanned_distance(SonarHead::Left)  > m_safe_distance)
        set_state(TURN);
    else
        set_state(REVERSE);
}
/*
void scan(Direction direction, uint16_t delay_ms)
{
    String log_string;
    switch(direction)
    {
    case LEFT:
        m_distance[direction] = m_sonar_head.scan_direction(SonarHead::Right, delay_ms);
        log_string = "Right: ";
        break;
    case FORWARD:
        m_distance[direction] = m_sonar_head.scan_direction(SonarHead::Forward, delay_ms);
        log_string = "Fwd  : ";
        break;
    case RIGHT:
        m_distance[direction] = m_sonar_head.scan_direction(SonarHead::Left, delay_ms);
        log_string = "Left : ";
        break;
    }
    
    log_string += String(m_distance[direction], 2) + "m     "; // 16
    Logger::instance().log(0, 1, log_string);

    delay(delay_ms);
}
*/
void turn(uint16_t duration)
{
    String log_string;
    if (m_sonar_head.scanned_distance(SonarHead::Right) > m_sonar_head.scanned_distance(SonarHead::Left))
    {
        m_motor.turn_right();
        
        log_string = "Right";
    }
    else
    {
        m_motor.turn_left();
        
        log_string = "Left ";
    }
    
    log_string += "           "; // 5 + 11
    Logger::instance().log(0, 1, log_string);

    delay(duration);
    m_motor.stop();

    set_state(ANALYZE);
}

void drive()
{
    float distance = m_sonar_head.scan_direction(SonarHead::Forward);
    
    String log_string;
    /*if (distance > m_safe_distance)
    {
        m_motor.forward();
        log_string += "Fast";
    }
    else*/ if (distance > m_stop_distance)
    {
        float speed_factor = map(distance, m_stop_distance, m_safe_distance, 0, 100) * 0.01f;
        speed_factor = 0.25f + (speed_factor * 0.75f);

        m_motor.forward(speed_factor);

        log_string += String(speed_factor, 3);
    }    
    else
    {
        m_motor.stop();
        set_state(ANALYZE);

        log_string += "-00-";
    }

    log_string += (" Dist: " + String(distance, 2) + "m");  // 12 chars
    Logger::instance().log(0, 1, log_string);
}

void reverse(uint16_t duration)
{
    m_motor.backward();
    delay(duration);
    m_motor.stop();

    // re-evaluate
    set_state(ANALYZE);
}

// ----------------------------------------------
void log_distance(uint16_t delay_ms)
{
    float distance = m_sonar_head.current_distance();
    Logger::instance().log(0, 1, "Dist: " + String(distance, 3) + " m");
    delay(delay_ms);
}

void log_sweep(uint16_t delay_ms)
{
    m_sonar_head.sweep(-45, 45, 3);
    
    String log_string;
    for (uint8_t i = 0; i < m_sonar_head.scan_count(); ++i)
        log_string += String(m_sonar_head.scanned_index(i), 2) + " ";
    Logger::instance().log(0, 1, log_string);

    delay(delay_ms);
}

// ----------------------------------------------
void loop()
{
    //log_distance(500);
    //log_sweep(500);
 
    switch(m_state)
    {
    case ANALYZE:   analyze(1000);  break;
    case TURN:      turn(500);      break;
    case DRIVE:     drive();        break;
    case REVERSE:   reverse(2000);  break;
    }
}