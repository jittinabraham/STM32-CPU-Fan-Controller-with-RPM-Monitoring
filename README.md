# **STM32 CPU Fan Controller with RPM Monitoring**

## **Project Overview**

This project involves creating a system to **control** and **monitor the RPM (Revolutions Per Minute)** of a fan using an **STM32 microcontroller**. The system continuously measures the fan's RPM and transmits the data via **UART** to a PC. The PC can visualize or log the RPM data. Additionally, the system can receive commands from the cloud (or PC via UART) to set the fan's RPM, adjusting the **PWM signal** accordingly.

---

### **Key Features**

- **RPM Measurement**: Continuously measures the fan's RPM using an input signal from the fan's tachometer.
- **UART Communication**: Transmits the measured RPM to a PC or cloud for monitoring and visualization.
- **PWM Control**: Adjusts the fan's RPM by changing the PWM signal based on incoming commands.
- **Cloud Integration**: Receives RPM set commands from the cloud or PC via UART and adjusts the fan speed.

---

## **Project Components**

### **Hardware**

- **STM32 Microcontroller (e.g., C031C6)**: The main controller responsible for managing RPM measurement, UART communication, and PWM signal generation.
- **Fan with Tachometer**: Provides a tachometer signal (pulse train) to measure the fan's RPM.
- **PWM Driver**: Generates a PWM signal to control the fan speed by adjusting the duty cycle.
- **UART Interface**: The STM32 uses UART to send and receive data from the PC or cloud.

### **Software**

- **FreeRTOS**: Used to manage two tasks:
  - **Task 1**: Periodically measures and transmits the current RPM of the fan to the PC.
  - **Task 2**: Receives RPM set commands and adjusts the PWM signal to control the fan speed.
- **HAL (Hardware Abstraction Layer)**: STM32 HAL libraries manage communication, GPIO configuration, and PWM generation.
- **STM32CubeMX**: Configures the STM32's peripherals and middleware (including FreeRTOS and PWM generation).

---

## **Project Workflow**

### **1. RPM Measurement**
- The microcontroller reads the tachometer signal using a timer interrupt to count pulses.
- RPM is calculated based on the number of pulses in a given time period.

### **2. UART Communication**
- The calculated RPM is transmitted to a PC via UART, allowing visualization, logging, or cloud integration.

### **3. Receiving RPM Set Commands**
- The microcontroller listens for commands like "SET RPM 1500" from the PC or cloud.
- Upon receiving the command, the microcontroller adjusts the PWM duty cycle to match the desired fan speed.

### **4. PWM Signal Generation**
- The microcontroller generates a PWM signal with an adjustable duty cycle to control the fan speed based on the received RPM command.

### **5. Cloud Integration**
- The PC or cloud-based application can send RPM commands to the STM32 via UART, allowing the system to adjust the PWM signal and fan speed.

---

## **System Architecture**

### **Task 1: RPM Measurement and UART Transmission**
This task reads the tachometer signal, calculates the RPM, and sends it over UART to the PC every second.

### **Task 2: UART Reception and PWM Adjustment**
This task listens for RPM set commands from the PC or cloud (e.g., "SET RPM 1500") and adjusts the PWM signal to control the fan speed.

---

## **Project Diagram**

![System Architecture Diagram](https://github.com/user-attachments/assets/031ba482-28a2-4084-8c77-fd4a3e7cfeaa)

---

## **Prerequisites**

To work with this project, ensure you have the following:

- **[STM32CubeIDE v1.11+](https://www.st.com/en/development-tools/stm32cubeide.html)**
- **STM32C0xx HAL Library**
- **STM32 Nucleo C0310C6**
- **CPU Exhaust Fan NB-BlacksilentPro**

---

## **Build Steps**

### **Step 1: Clone the Repository**

Clone the project repository to your local machine:

```bash
git clone https://github.com/yourusername/STM32-CPU-Fan-Controller-with-RPM-Monitoring.git
