#include <Bluepad32.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>
#include "esp_bt.h"

// ========== КОНСТАНТЫ И НАСТРОЙКИ ==========
const int escPinRear = 17;      //ESC задний
const int escPinFront = 16;     //ESC передний
const int servoPin = 18;        //Рулевая
const int servoPin2 = 19;
const int servoPin3 = 21;

const float BATTERY_FULL_V = 8.40;
const float BATTERY_EMPTY_V = 6.80;
const uint8_t BNO055_READ_INTERVAL = 10; // мс

// Адреса EEPROM
enum EEPROMAddresses {
    EEPROM_TRIM_ADDR = 0
};

// Триммирование
const int8_t TRIM_STEP = 2;
const int8_t MAX_TRIM = 30;
int8_t steeringTrim = 0; //Калибровка 0 рулевой

// Сервопривод
const int8_t servoCenter = 90;
const int8_t servoRange = 60;

// ESC
const uint16_t escNeutral = 1500;
const uint16_t escMaxForward = 2000;
const uint16_t escMaxReverse = 1000;

// Вольтметр
const uint8_t voltagePin1 = 34;
const uint8_t voltagePin2 = 35;
const float voltageDividerRatio = 11;              // Коэффициент делителя: K = (R1 + R2) / R2, Пример: для R1=100кОм, R2=33кОм -> K = 133 / 33 ≈ 4.03
const float adcReferenceVoltage = 3.3f;             // Опорное напряжение АЦП ESP32 (обычно 3.3В)
const uint16_t adcResolution = 4096;                // Разрешение АЦП (12 бит = 4096)
const uint32_t BATTERY_CHECK_INTERVAL = 15000;      // мс

// Стабилизация
const uint16_t STABILIZATION_DURATION = 1500;       //Подолжэительность стабилизации
const float STABILIZATION_ACCEL_THRESHOLD = 1.8f;
const uint16_t STABILIZATION_GAS_THRESHOLD = 200;

// ========== СТРУКТУРЫ ДАННЫХ ==========
//Ключевые моменты реализации
//Приоритет управления: Руль от PS4 (steeringFromPS4) — основной сигнал. Коррекция стабилизации (stabilizationCompensation) лишь добавляется к нему. Это гарантирует, что команда на поворот будет выполнена.
//Условие активации: Стабилизация включается только при одновременном выполнении условий: ускорение > 1.8 м/с² И нажатие на газ (abs(stickY) > 50). Это предотвращает ложные срабатывания при качении машины по инерции.
//Ограничение сигнала: Функция constrain() после смешивания защищает сервопривод от получения опасных ШИМ-значений.
//Настройка PID: Начните с предложенных коэффициентов (Kp=1.2, Ki=0.02, Kd=0.4). Если машина при разгоне колеблется из стороны в сторону — уменьшите Kp и увеличьте Kd. Если плохо держит курс — увеличьте Kp.
struct PIDController {
    float Kp = 1.2f;        // Основная реакция на занос
    float Ki = 0.02f;       // Плавное устранение остаточной ошибки
    float Kd = 0.4f;         // Демпфирование резких рывков
    float integral = 0.0f;
    float prev_error = 0.0f;
    float max_integral = 50.0f;
    float last_output = 0.0f;
};
// ===================== ПЕРЕМЕННЫЕ ДЛЯ СТАБИЛИЗАЦИИ =====================
struct VehicleState {
    float currentYaw = 0.0f;
    float targetYaw = 0.0f;
    float forwardAccel = 0.0f;
    bool isStabilizationActive = false;
    uint32_t stabilizationStartTime = 0;
    int16_t steeringFromPS4 = servoCenter;
    uint16_t throttleRear = escNeutral;
    uint16_t throttleFront = escNeutral;
    int16_t finalSteering = servoCenter;
};
// ===================== ПЕРЕМЕННЫЕ КНОПОК КОНТРОЛЛЕРА =====================
struct ControllerState {
    bool l1 = false;
    bool r1 = false;
    int16_t axisX = 0;
    uint16_t throttle = 0;
    uint16_t brake = 0;
    bool isConnected = false;
};

// ========== ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ==========
ControllerPtr myController;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

Servo steeringServo;
Servo firstServo;       //НА БУДУЩЕЕ
Servo secondServo;      //НА БУДУЩЕЕ
Servo escRear;
Servo escFront;

PIDController yawPID;
VehicleState vehicle;
ControllerState controller;

// ========== ПРОТОТИПЫ ФУНКЦИЙ ==========
void onConnectedController(ControllerPtr ctl);
void onDisconnectedController(ControllerPtr ctl);
void updateControllerState();
void updateBNO055();
void processControllerInput();
void processStabilization();
void applyControlSignals();
void checkBattery();
void debugOutput();
void saveTrimAndApply();
float calculatePID(float input, float setpoint, float dt);
void setControllerColor(uint8_t r, uint8_t g, uint8_t b);

// ========== ОСНОВНЫЕ ФУНКЦИИ ==========
void setup() {
    Serial.begin(115200);
    
    // Инициализация Bluepad32
    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys();
    // Установка мощности Bluetooth (от 0 до 9, где 9 - максимум)
    // WARNING: Высокая мощность увеличивает нагрев и расход батареи!
    esp_power_level_t power = ESP_PWR_LVL_P9; // Максимум
    esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);

    // Инициализация EEPROM и загрузка трима
    EEPROM.begin(512);
    EEPROM.get(EEPROM_TRIM_ADDR, steeringTrim);
    if (steeringTrim < -MAX_TRIM || steeringTrim > MAX_TRIM) {
        steeringTrim = 0;
        EEPROM.put(EEPROM_TRIM_ADDR, steeringTrim);
        EEPROM.commit();
    }
    
    // Инициализация сервоприводов
    steeringServo.attach(servoPin);
    steeringServo.write(servoCenter + steeringTrim);
    
    // Инициализация ESC
    escRear.attach(escPinRear);
    escFront.attach(escPinFront);
    escRear.writeMicroseconds(escNeutral);
    escFront.writeMicroseconds(escNeutral);
    delay(4000); // Инициализация ESC
    
    // Инициализация BNO055
    Wire.begin(21, 22);
    if (!bno.begin()) {
        Serial.println("Ошибка BNO055!");
    }
    delay(1000);
    bno.setExtCrystalUse(true);
    
    Serial.println("Система инициализирована");
}

void loop() {
    static uint32_t lastLoopTime = 0;
    static uint32_t lastBNORead = 0;
    static uint32_t lastBatteryCheck = 0;
    static uint32_t lastDebugOutput = 0;
    
    uint32_t currentTime = millis();
    
    // 1. Обновление контроллера (высокий приоритет)
    BP32.update();
    
    // 2. Основной цикл с фиксированным интервалом (10мс)
    if (currentTime - lastLoopTime >= 10) {
        lastLoopTime = currentTime;
        
        // 2.1 Обновление состояния контроллера
        updateControllerState();
        
        // 2.2 Обновление датчиков (каждые 10мс)
        if (currentTime - lastBNORead >= 10) {
            updateBNO055();
            lastBNORead = currentTime;
        }
        
        // 2.3 Обработка ввода
        if (controller.isConnected) {
            processControllerInput();
        }
        
        // 2.4 Обработка стабилизации
        processStabilization();
        
        // 2.5 Применение управляющих сигналов
        applyControlSignals();
        
        // 2.6 Отладочный вывод (каждые 100мс)
        if (currentTime - lastDebugOutput >= 100) {
            debugOutput();
            lastDebugOutput = currentTime;
        }
    }
    
    // 3. Проверка батареи (каждые 15 секунд)
    if (currentTime - lastBatteryCheck >= BATTERY_CHECK_INTERVAL) {
        checkBattery();
        lastBatteryCheck = currentTime;
    }
}

// ========== РЕАЛИЗАЦИЯ ФУНКЦИЙ ==========
void onConnectedController(ControllerPtr ctl) {
    myController = ctl;
    controller.isConnected = true;
    Serial.println("Контроллер подключен");
}

void onDisconnectedController(ControllerPtr ctl) {
    if (myController == ctl) {
        myController = nullptr;
        controller.isConnected = false;
        Serial.println("Контроллер отключен");
        
        // Безопасное отключение
        escRear.writeMicroseconds(escNeutral);
        escFront.writeMicroseconds(escNeutral);
        steeringServo.write(servoCenter + steeringTrim);
    }
}

void updateControllerState() {
    if (!myController || !myController->isConnected()) {
        controller.isConnected = false;
        return;
    }
    
    controller.isConnected = true;
    controller.l1 = myController->l1();
    controller.r1 = myController->r1();
    controller.axisX = myController->axisX();
    controller.throttle = myController->throttle();
    controller.brake = myController->brake();
}

void updateBNO055() {
    sensors_event_t orientationData, linearAccelData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    
    vehicle.currentYaw = orientationData.orientation.x;
    vehicle.forwardAccel = linearAccelData.acceleration.y;
}

void processControllerInput() {
    static bool lastL1 = false, lastR1 = false;
    
    // 1. Управление рулем
    vehicle.steeringFromPS4 = servoCenter + 
                             map(controller.axisX, -512, 512, -servoRange, servoRange) + 
                             steeringTrim;
    
    // 2. Управление газом/тормозом
    vehicle.throttleRear = escNeutral;
    vehicle.throttleFront = escNeutral;
    
    if (controller.throttle > 20) {
        vehicle.throttleRear = map(controller.throttle, 20, 1023, escNeutral, escMaxForward);
        
        // Режим усиления передних колес при ускорении
        if (vehicle.forwardAccel > 2.0f) {
            int16_t boost = (vehicle.throttleRear - escNeutral) / 4; // +25%
            vehicle.throttleFront = constrain(vehicle.throttleRear + boost, escNeutral, escMaxForward);
        } else {
            vehicle.throttleFront = vehicle.throttleRear;
        }
    } else if (controller.brake > 20) {
        vehicle.throttleRear = map(controller.brake, 20, 1023, escNeutral, escMaxReverse);
        vehicle.throttleFront = vehicle.throttleRear; // Одинаковое торможение
    }
    
    // 3. Триммирование руля
    if (controller.l1 && !lastL1) {
        steeringTrim -= TRIM_STEP;
        steeringTrim = max(steeringTrim, (int8_t)-MAX_TRIM); // 
        saveTrimAndApply();
    }
    
    if (controller.r1 && !lastR1) {
        steeringTrim += TRIM_STEP;
        steeringTrim = min(steeringTrim, (int8_t)MAX_TRIM);  // 
        saveTrimAndApply();
    }
    
    lastL1 = controller.l1;
    lastR1 = controller.r1;
    
    // 4. Активация стабилизации
    if (!vehicle.isStabilizationActive &&
        vehicle.forwardAccel > STABILIZATION_ACCEL_THRESHOLD &&
        controller.throttle > STABILIZATION_GAS_THRESHOLD) {
        vehicle.targetYaw = vehicle.currentYaw;
        vehicle.isStabilizationActive = true;
        vehicle.stabilizationStartTime = millis();
        yawPID.integral = 0;
        yawPID.prev_error = 0;
    }
}

void processStabilization() {
    if (!vehicle.isStabilizationActive) {
        return;
    }
    
    // Проверка таймаута
    if (millis() - vehicle.stabilizationStartTime > STABILIZATION_DURATION) {
        vehicle.isStabilizationActive = false;
        return;
    }
    
    // Расчет PID
    float dt = 0.01f; // 10мс в секундах
    float compensation = calculatePID(vehicle.currentYaw, vehicle.targetYaw, dt);
    
    // Применение компенсации
    vehicle.finalSteering = vehicle.steeringFromPS4 + (int16_t)compensation;
    vehicle.finalSteering = constrain(vehicle.finalSteering,
                                     servoCenter - servoRange,
                                     servoCenter + servoRange);
}

float calculatePID(float input, float setpoint, float dt) {
    float error = setpoint - input;
    
    // Коррекция ошибки по углу
    if (error > 180.0f) error -= 360.0f;
    else if (error < -180.0f) error += 360.0f;
    
    // Пропорциональная часть
    float p_out = yawPID.Kp * error;
    
    // Интегральная часть
    yawPID.integral += error * dt;
    yawPID.integral = constrain(yawPID.integral, -yawPID.max_integral, yawPID.max_integral);
    float i_out = yawPID.Ki * yawPID.integral;
    
    // Дифференциальная часть
    float derivative = (error - yawPID.prev_error) / dt;
    float d_out = yawPID.Kd * derivative;
    yawPID.prev_error = error;
    
    return p_out + i_out + d_out;
}

void applyControlSignals() {
    static int16_t lastSteering = 0;
    static uint16_t lastThrottleRear = 0;
    static uint16_t lastThrottleFront = 0;
    
    // Обновляем только при изменении значений
    if (vehicle.finalSteering != lastSteering) {
        steeringServo.write(vehicle.finalSteering);
        lastSteering = vehicle.finalSteering;
    }
    
    if (vehicle.throttleRear != lastThrottleRear) {
        escRear.writeMicroseconds(vehicle.throttleRear);
        lastThrottleRear = vehicle.throttleRear;
    }
    
    if (vehicle.throttleFront != lastThrottleFront) {
        escFront.writeMicroseconds(vehicle.throttleFront);
        lastThrottleFront = vehicle.throttleFront;
    }
}

void checkBattery() {
    int adcValue1 = analogRead(voltagePin1);
    int adcValue2 = analogRead(voltagePin2);
    
    float batteryVoltage1 = (adcValue1 * adcReferenceVoltage / adcResolution) * voltageDividerRatio;
    float batteryVoltage2 = (adcValue2 * adcReferenceVoltage / adcResolution) * voltageDividerRatio;
    
    float minBatteryVoltage = min(batteryVoltage1, batteryVoltage2);
    
    // Индикация на контроллере
    if (controller.isConnected) {
        float clampedVoltage = constrain(minBatteryVoltage, BATTERY_EMPTY_V, BATTERY_FULL_V);
        float normalizedLevel = (clampedVoltage - BATTERY_EMPTY_V) / (BATTERY_FULL_V - BATTERY_EMPTY_V);
        
        uint8_t red = 255 - (uint8_t)(normalizedLevel * 255);
        uint8_t green = 0;
        if (normalizedLevel < 0.5f) {
            green = (uint8_t)(normalizedLevel * 2 * 255);
        } else {
            green = (uint8_t)((1.0f - normalizedLevel) * 2 * 255);
        }
        uint8_t blue = (uint8_t)(normalizedLevel * 255);
        
        setControllerColor(red, green, blue);
        
        // Вибрация при низком заряде
        if (minBatteryVoltage < BATTERY_EMPTY_V + 0.30f) {
            myController->playDualRumble(10, 150, 0x40, 0x40);
            Serial.println("ВНИМАНИЕ: Низкий заряд аккумулятора!");
        }
    }
    
    Serial.print("АКБ1: ");
    Serial.print(batteryVoltage1);
    Serial.print("В | АКБ2: ");
    Serial.print(batteryVoltage2);
    Serial.println("В");
}

void setControllerColor(uint8_t r, uint8_t g, uint8_t b) {
    if (myController && controller.isConnected) {
        myController->setColorLED(r, g, b);
    }
}

void saveTrimAndApply() {
    EEPROM.put(EEPROM_TRIM_ADDR, steeringTrim);
    EEPROM.commit();
    
    // Немедленное применение
    steeringServo.write(servoCenter + steeringTrim);
    
    Serial.print("Трим сохранен: ");
    Serial.print(steeringTrim);
    Serial.println(" град.");
}

void debugOutput() {
    Serial.print("Руль: ");
    Serial.print(vehicle.steeringFromPS4);
    Serial.print(" | Итог: ");
    Serial.print(vehicle.finalSteering);
    Serial.print(" | ГазЗ: ");
    Serial.print(vehicle.throttleRear);
    Serial.print(" | ГазП: ");
    Serial.print(vehicle.throttleFront);
    Serial.print(" | Yaw: ");
    Serial.print(vehicle.currentYaw, 1);
    Serial.print(" | Ускор: ");
    Serial.print(vehicle.forwardAccel, 2);
    Serial.print(" | Стаб: ");
    Serial.println(vehicle.isStabilizationActive ? "ВКЛ" : "ВЫКЛ");
}
