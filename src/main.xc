#include <CORE_C22-rev-a.bsp>
#include <IFM_DC1K-rev-c4.bsp>

#include <pwm_server.h>
#include <pwm_ports.h>
#include <adc_service.h>
#include <user_config.h>
#include <position_feedback_service.h>
#include <xscope.h>
#include <stdio.h>
#include <print.h>
#include <string.h>
#include <platform.h>
#include <motor_control_interfaces.h>
#include <advanced_motor_control.h>
#include <refclk.h>
#include <motion_control_service.h>

PwmPorts pwm_ports = SOMANET_IFM_PWM_PORTS;
WatchdogPorts wd_ports = SOMANET_IFM_WATCHDOG_PORTS;
FetDriverPorts fet_driver_ports = SOMANET_IFM_FET_DRIVER_PORTS;
ADCPorts adc_ports = SOMANET_IFM_ADC_PORTS;
QEIHallPort qei_hall_port_1 = SOMANET_IFM_HALL_PORTS;
QEIHallPort qei_hall_port_2 = SOMANET_IFM_QEI_PORTS;
HallEncSelectPort hall_enc_select_port = SOMANET_IFM_QEI_PORT_INPUT_MODE_SELECTION;
SPIPorts spi_ports = SOMANET_IFM_SPI_PORTS;
port ?gpio_port_0 = SOMANET_IFM_GPIO_D0;
port ?gpio_port_1 = SOMANET_IFM_GPIO_D1;
port ?gpio_port_2 = SOMANET_IFM_GPIO_D2;
port ?gpio_port_3 = SOMANET_IFM_GPIO_D3;

port rs485_transmit = COM_PORT_1BIT_18;
port rs485_receive = COM_PORT_1BIT_14;
port rs485_status = COM_PORT_1BIT_58;
port button = COM_PORT_1BIT_10;

int MAX_NUMBER_OF_ATTEMPTS=10; // The maximum number of attempts to communicate with the handle before an error is raised

//  Sets the status of the line :
//     0 -> The somanet listens to what is happening on the line, the line is in idle mode
//     1 -> The somanet can write on the line, but stops reading in order not to read its own messages
void set_line_status(int status)
{
    rs485_status <: status;
}

// Sets the Tx pin of the transceiver to the value of the required bit.
// Will have no effect if the transmission is not enabled. (set_line_status(1))
void send_bit(int bit)
{
    rs485_transmit <: 1-bit; // For some reason , a positive voltage on the RS485 line seems to be a logic 0...
}

// Polls the Rx pin of the transceiver.
// Will have no effect if the reception is not enabled. (set_line_status(0))
int receive_bit()
{
    int res;
    rs485_receive :> res;
    return 1-res;
}

// This interface is used to overwrite the buffer containing the bits that should be sent.
// Warning, overwriting this buffer will cancel all activity on the line (transmission and reception)
interface  rs485_interface {
    void send_message(int buffer[],int buffer_size);
};

// This interface is used to indicate when a message containing the position of the switch has been received
interface switch_feedback {
    void switch_input(int integer);
};

// This interface is used to communicate the status of the button to the behavior function
interface button_interface {
    void button_input(int integer);
};

// This interface is used to make sure that the handle has taken into account the write requests that the somanet sends.
interface LED_feedback {
    void success();
};

// This function tells which character corresponds to a 10 bit buffer.
// It returns -1 when the character is not valid
int compute_listening_buffer(int * buffer)
{
    int res = -1;
    if(buffer[0]==0 && buffer[9]==1)
    {
        res = buffer[1]+2*buffer[2]+4*buffer[3]+8*buffer[4]+16*buffer[5]+32*buffer[6]+64*buffer[7];
    }
    return(res);
}

// This function gives the char corresponding to an int (the int being the ascii value of this character ex : 48 -> 0)
char int_to_char_ascii(int n)
{
    char res;
    switch(n)
            {
            case 48: //0
                        res = '0';
                        break;
            case 49: //1
                        res = '1';
                            break;
            case 50: //2
                        res = '2';
                            break;
            case 51: //3
                        res = '3';
                        break;
            case 52: //4
                        res = '4';
                        break;
            case 53: //5
                        res = '5';
                        break;
            case 54: //6
                        res = '6';
                        break;
            case 55: //7
                        res = '7';
                        break;
            case 56: //8
                        res = '8';
                        break;
            case 57: //9
                        res = '9';
                        break;
            case 65: //A
                        res = 'A';
                        break;
            case 66: //B
                        res = 'B';
                        break;
            case 67: //C
                        res = 'C';
                        break;
            case 68: //D
                        res = 'D';
                        break;
            case 69: //E
                        res = 'E';
                        break;
            case 70: //F
                        res = 'F';
                        break;
            case 58: //:
                        res = ':';
                        break;
            case 13: //CR
                        res = 'R';
                        break;
            case 10: //LF
                        res = 'L';
                        break;
            default:
                        res = 'N';
                        break;
            }
    return(res);
}

// This function gives the char corresponding to an int (the int being the hexadecimal value of this character, ex : 15 -> F)
char int_to_char_hex(int n)
{
    char res;
    switch(n)
            {
            case 0: //0
                        res = '0';
                        break;
            case 1: //1
                        res = '1';
                            break;
            case 2: //2
                        res = '2';
                            break;
            case 3: //3
                        res = '3';
                        break;
            case 4: //4
                        res = '4';
                        break;
            case 5: //5
                        res = '5';
                        break;
            case 6: //6
                        res = '6';
                        break;
            case 7: //7
                        res = '7';
                        break;
            case 8: //8
                        res = '8';
                        break;
            case 9: //9
                        res = '9';
                        break;
            case 10: //A
                        res = 'A';
                        break;
            case 11: //B
                        res = 'B';
                        break;
            case 12: //C
                        res = 'C';
                        break;
            case 13: //D
                        res = 'D';
                        break;
            case 14: //E
                        res = 'E';
                        break;
            case 15: //F
                        res = 'F';
                        break;
            default:
                        res = '0';
                        break;
            }
    return(res);
}

// This function returns the integer corresponding to the hexadecimal value of a character (ex : A -> 10)
int char_to_int(char character)
{
    int integer_value = (int)character;
    if(integer_value>47 && integer_value<58)
    {
        return(integer_value-48);
    }
    else if(integer_value>64 && integer_value<71)
    {
        return(integer_value-55);
    }
    else
    {
        return(0);
    }
}

// This function stores the binary value that should be sent on the RS485 line to send a character (given its ascii integer) in an array
void char_to_bin(int character,int * bin)
{
    switch(character)
    {
    case 48: // 0
            int res[11] = {0,0,0,0,0,1,1,0,0,0,1};
            int i;
                for(i=0;i<11;i++)
                {
                    bin[i] = res[i];
                }
            break;
    case 49: // 1
            int res[11] = {0,1,0,0,0,1,1,0,0,0,1};
            int i;
                for(i=0;i<11;i++)
                {
                    bin[i] = res[i];
                }
            break;
    case 50:
            int res[11] = {0,0,1,0,0,1,1,0,0,0,1};
            int i;
                for(i=0;i<11;i++)
                {
                    bin[i] = res[i];
                }
            break;
    case 51:
            int res[11] = {0,1,1,0,0,1,1,0,0,0,1};
            int i;
                for(i=0;i<11;i++)
                {
                    bin[i] = res[i];
                }
            break;
    case 52:
            int res[11] = {0,0,0,1,0,1,1,0,0,0,1};
            int i;
                for(i=0;i<11;i++)
                {
                    bin[i] = res[i];
                }
            break;
    case 53:
            int res[11] = {0,1,0,1,0,1,1,0,0,0,1};
            int i;
                for(i=0;i<11;i++)
                {
                    bin[i] = res[i];
                }
            break;
    case 54:
            int res[11] = {0,0,1,1,0,1,1,0,0,0,1};
            int i;
                for(i=0;i<11;i++)
                {
                    bin[i] = res[i];
                }
            break;
    case 55:
            int res[11] = {0,1,1,1,0,1,1,0,0,0,1};
            int i;
                for(i=0;i<11;i++)
                {
                    bin[i] = res[i];
                }
            break;
    case 56:
            int res[11] = {0,0,0,0,1,1,1,0,0,0,1};
            int i;
                for(i=0;i<11;i++)
                {
                    bin[i] = res[i];
                }
            break;
    case 57:
            int res[11] = {0,1,0,0,1,1,1,0,0,0,1};
            int i;
                for(i=0;i<11;i++)
                {
                    bin[i] = res[i];
                }
            break;
    case 65:
            int res[11] = {0,1,0,0,0,0,0,1,0,0,1};
            int i;
                for(i=0;i<11;i++)
                {
                    bin[i] = res[i];
                }
            break;
    case 66:
            int res[11] = {0,0,1,0,0,0,0,1,0,0,1};
            int i;
                for(i=0;i<11;i++)
                {
                    bin[i] = res[i];
                }
            break;
    case 67:
            int res[11] = {0,1,1,0,0,0,0,1,0,0,1};
            int i;
                for(i=0;i<11;i++)
                {
                    bin[i] = res[i];
                }
            break;
    case 68:
            int res[11] = {0,0,0,1,0,0,0,1,0,0,1};
            int i;
                for(i=0;i<11;i++)
                {
                    bin[i] = res[i];
                }
            break;
    case 69:
            int res[11] = {0,1,0,1,0,0,0,1,0,0,1};
            int i;
                for(i=0;i<11;i++)
                {
                    bin[i] = res[i];
                }
            break;
    case 70:
            int res[11] = {0,0,1,1,0,0,0,1,0,0,1};
            int i;
                for(i=0;i<11;i++)
                {
                    bin[i] = res[i];
                }
            break;
    case 58:
            int res[11] = {0,0,1,0,1,1,1,0,0,0,1};
            int i;
                for(i=0;i<11;i++)
                {
                    bin[i] = res[i];
                }
            break;
    case 82:
            int res[11] = {0,1,0,1,1,0,0,0,0,0,1};
            int i;
                for(i=0;i<11;i++)
                {
                    bin[i] = res[i];
                }
            break;
    case 76:
            int res[11] = {0,0,1,0,1,0,0,0,0,0,1};
            int i;
                for(i=0;i<11;i++)
                {
                    bin[i] = res[i];
                }
            break;
    default:
            int res[11] = {1,1,1,1,1,1,1,1,1,1,1};
            int i;
                for(i=0;i<11;i++)
                {
                    bin[i] = res[i];
                }
            break;
    }
}

// This function generates the complete binary sequence that should be sent over the RS485 bus to send an ascii message.
void generate_message(char * character_message, int size, int * bin_message)
{
    bin_message[0] = 1;
    int i;
    int j;
    for(i=0;i<size;i++)
    {
        int character = character_message[i];
        int bin[11] = {};
        char_to_bin(character,(int *)bin);
        for(j=0;j<11;j++)
        {
            int pos = 1+11*i+j;
            bin_message[pos] = bin[j];
        }
    }
}

// This functions computes the LRC associated to an ascii message.
{int ,int} compute_LRC(char * message)
{
    int pos = 1;
    int sum = 0;
    while((char)message[pos+2]!='R')
    {
        int char_0  = message[pos+1];
        int char_1  = message[pos];
        int value_0 = 0;
        int value_1 = 0;
        if(char_0 > 47 && char_0 < 58)
        {
            value_0 = char_0-48;
        }
        else if (char_0 > 64 && char_0 < 71)
        {
            value_0 = char_0-55;
        }
        if(char_1 > 47 && char_1 < 58)
        {
            value_1 = char_1-48;
        }
        else if (char_1 > 64 && char_1 < 71)
        {
            value_1 = char_1-55;
        }
        sum = sum+value_0+16*value_1;
        pos = pos+2;
    }
    while(sum>255)
    {
        sum = sum-256;
    }
    if(sum==0)
    {
        sum = 256;
    }
    int res = 255-sum+1;
    int LRC_1 = (int)res/16;
    int LRC_0 = res;
    while(LRC_0 > 15)
    {
        LRC_0 = LRC_0-16;
    }
    if(LRC_0<10)
    {
        LRC_0 = LRC_0+48;
    }
    else
    {
        LRC_0 = LRC_0+55;
    }
    if(LRC_1<10)
    {
        LRC_1 = LRC_1+48;
    }
    else
    {
        LRC_1 = LRC_1+55;
    }
    return {LRC_0,LRC_1};
}

// This function checks whether a received message is valid (starts with ":", ends with RL and has a valid LRC) and then uses interfaces to communicate the message to the behavior function
void compute_message(int * character_buffer, client interface switch_feedback i_switch_feedback, client interface LED_feedback j)
{
    int over = 0;
    int pos = 0;
    int LRC_pos = 0;
    char message[1000] = {};
    int valid = 0;
    while(over==0)
    {
        char character = int_to_char_ascii(character_buffer[pos]);
        message[pos] = character;
        pos++;
        if(character=='L')
        {
            over = 1;
            if(message[pos-2]=='R' && message[0]==':')
            {
                valid = 1;
                LRC_pos = pos-4;
            }
            else
            {
                valid = 0;
            }
        }
    }
    if(valid == 1)
    {
        valid = 0;
        int given_LRC_0 = (int)message[LRC_pos+1];
        int given_LRC_1 = (int)message[LRC_pos];
        int check_LRC_0;
        int check_LRC_1;
        {check_LRC_0,check_LRC_1} = compute_LRC(message);
        if(given_LRC_0== check_LRC_0 || given_LRC_1== check_LRC_1)
        {
            valid = 1;
        }
    }
    if(valid == 1)
    {
        if(message[4]=='4')// If this is a response from a read request
        {
            int integer_value_0 = char_to_int(message[10]);
            int integer_value_1 = char_to_int(message[9]);
            int integer_value_2 = char_to_int(message[8]);
            int val = integer_value_0 + 16*integer_value_1 + 16*16*integer_value_2;
            while(val>=2048)
            {
                val = val - 2048;
            }
            while(val>=1024)
                {
                    val = val - 1024;
                }
            val = (int)100*(val-106)/(930-106);
            switch((char)message[7])
            {
            case '0':
                val = 0;
                break;
            case '1':
                break;
            case '2':
                val = -val;
                break;
            }
            if(val*val<10000)
            {
                par
                {
                    i_switch_feedback.switch_input(val);
                }
            }
        }
        if(message[4]=='6')// If this is a response from a write request
        {
             {
                 j.success();
             }
        }
    }
}

// This function writes and listens to what is happening on the RS485 bus.
void handle_rs485(int baud_rate,server interface rs485_interface i_rs485_interface,client interface switch_feedback j,client interface LED_feedback k)
{

    int line_status = 0; // 0 -> listening, 1 -> writing
    set_line_status(0);

    int sending_buffer[1000] = {};
    int sending_buffer_size = 0;
    int pos_in_sending_buffer = 0;

    int listening_buffer[10] = {};
    int pos_in_listening_buffer = 0;
    int characters_buffer[1000] = {};
    int pos_in_characters_buffer = 0;

    timer tmr;
    unsigned  time;
    tmr :> time;

    while(1)
    {
        select {
            case i_rs485_interface.send_message(int new_buffer[], int new_buffer_size):
            // In case there is a request to be sent
                pos_in_sending_buffer=0;
                sending_buffer_size = new_buffer_size;
                int j;
                for(j=0; j<new_buffer_size;j=j+1)
                {
                    sending_buffer[j] = new_buffer[j];
                }
                line_status=1;
                set_line_status(1);
                break;
            case  tmr  when  timerafter(time) :> int  now:
            // In case of a "clock tick"
                time += (int)100000000/baud_rate;
                if(line_status==1)
                // If we are writing
                {
                    if(sending_buffer[pos_in_sending_buffer]==1)
                    {
                        send_bit(1);
                    }
                    else
                    {
                        send_bit(0);
                    }
                    pos_in_sending_buffer++;
                    if(pos_in_sending_buffer>=sending_buffer_size)
                    {
                        pos_in_sending_buffer=0;
                        line_status=0; //set the status to listening
                        set_line_status(0);
                    }
                }
                else
                // If we are listening
                {
                    int bit_received = receive_bit();
                    if(pos_in_listening_buffer==0)
                    {
                        pos_in_listening_buffer = pos_in_listening_buffer+(1-bit_received);
                    }
                    else
                    {
                        listening_buffer[pos_in_listening_buffer] = bit_received;
                        pos_in_listening_buffer++;
                        if(pos_in_listening_buffer>9)
                        {
                            pos_in_listening_buffer = 0;
                            int received_character = compute_listening_buffer(listening_buffer);

                            if(pos_in_characters_buffer!=0)
                            {
                                if(received_character == 10 || pos_in_characters_buffer>=1000)
                                {
                                    if(received_character == 10)
                                    {
                                        characters_buffer[pos_in_characters_buffer] = 10;
                                        compute_message(characters_buffer,j,k);
                                    }
                                    pos_in_characters_buffer = 0;
                                }
                                else
                                {
                                    characters_buffer[pos_in_characters_buffer] = received_character;
                                    pos_in_characters_buffer++;
                                }
                            }
                            if(received_character == 58)
                            {
                                 characters_buffer[0]=58;
                                 pos_in_characters_buffer = 1;
                            }
                        }
                    }
                }

                break;
        }
    }
}


// TODO : This function should unfold the wheel
void unfold_wheel(int app_tile_usec,
        int dc_bus_voltage,
        int pull_brake_voltage,
        int hold_brake_voltage,
        int pull_brake_time,
        client interface TorqueControlInterface i_torque_control, client interface UpdateBrake i_update_brake)
{
    printf("unfolding wheel... \n");
    int error=0;
    int duty_min=0, duty_max=0, duty_divider=0;
    int duty_start_brake =0, duty_maintain_brake=0, period_start_brake=0;

    timer t;
    unsigned ts;

    i_torque_control.set_safe_torque_off_enabled();
    i_torque_control.set_brake_status(0);
    t :> ts;
    t when timerafter (ts + 2000*1000*app_tile_usec) :> void;

    if(dc_bus_voltage <= 0)
    {
        printstr("ERROR: NEGATIVE VDC VALUE DEFINED IN SETTINGS");
        return;
    }

    if(pull_brake_voltage > (dc_bus_voltage*1000))
    {
        printstr("ERROR: PULL BRAKE VOLTAGE HIGHER THAN VDC");
        return;
    }

    if(pull_brake_voltage < 0)
    {
        printstr("ERROR: NEGATIVE PULL BRAKE VOLTAGE");
        return;
    }

    if(hold_brake_voltage > (dc_bus_voltage*1000))
    {
        printstr("ERROR: HOLD BRAKE VOLTAGE HIGHER THAN VDC");
        return;
    }

    if(hold_brake_voltage < 0)
    {
        printstr("ERROR: NEGATIVE HOLD BRAKE VOLTAGE");
        return;
    }

    if(period_start_brake < 0)
    {
        printstr("ERROR: NEGATIVE PERIOD START BRAKE SETTINGS!");
        return;
    }

    MotorcontrolConfig motorcontrol_config = i_torque_control.get_config();

    if(motorcontrol_config.ifm_tile_usec==250)
    {
        duty_min = 1500;
        duty_max = 13000;
        duty_divider = 16384;
    }
    else if(motorcontrol_config.ifm_tile_usec==100)
    {
        duty_min = 600;
        duty_max = 7000;
        duty_divider = 8192;
    }
    else if (motorcontrol_config.ifm_tile_usec!=100 && motorcontrol_config.ifm_tile_usec!=250)
    {
        error = 1;
    }

    duty_start_brake    = (duty_divider * pull_brake_voltage)/(1000*dc_bus_voltage);
    if(duty_start_brake < duty_min) duty_start_brake = duty_min;
    if(duty_start_brake > duty_max) duty_start_brake = duty_max;

    duty_maintain_brake = (duty_divider * hold_brake_voltage)/(1000*dc_bus_voltage);
    if(duty_maintain_brake < duty_min) duty_maintain_brake = duty_min;
    if(duty_maintain_brake > duty_max) duty_maintain_brake = duty_max;

    period_start_brake  = (pull_brake_time * 1000)/(motorcontrol_config.ifm_tile_usec);

    i_update_brake.update_brake_control_data(duty_start_brake, duty_maintain_brake, period_start_brake);

    // send voltage to lifting motor
    i_torque_control.set_brake_status(1);
    int unfold_time = 700; // chosen based on experimenting with the lifting motor
    delay_milliseconds(unfold_time);
    i_torque_control.set_brake_status(0);
    printf("wheel unfolded \n");

}

// TODO : This function should fold the wheel

void fold_wheel(int app_tile_usec,
        int dc_bus_voltage,
        int pull_brake_voltage,
        int hold_brake_voltage,
        int pull_brake_time,
        client interface TorqueControlInterface i_torque_control, client interface UpdateBrake i_update_brake)
{
    printf("folding wheel... \n");
    int error=0;
    int duty_min=0, duty_max=0, duty_divider=0;
    int duty_start_brake =0, duty_maintain_brake=0, period_start_brake=0;

    timer t;
    unsigned ts;

    i_torque_control.set_safe_torque_off_enabled();
    i_torque_control.set_brake_status(0);
    t :> ts;
    t when timerafter (ts + 2000*1000*app_tile_usec) :> void;

    if(dc_bus_voltage <= 0)
    {
        printstr("ERROR: NEGATIVE VDC VALUE DEFINED IN SETTINGS");
        return;
    }

    if(pull_brake_voltage > (dc_bus_voltage*1000))
    {
        printstr("ERROR: PULL BRAKE VOLTAGE HIGHER THAN VDC");
        return;
    }

    if(pull_brake_voltage < 0)
    {
        printstr("ERROR: NEGATIVE PULL BRAKE VOLTAGE");
        return;
    }

    if(hold_brake_voltage > (dc_bus_voltage*1000))
    {
        printstr("ERROR: HOLD BRAKE VOLTAGE HIGHER THAN VDC");
        return;
    }

    if(hold_brake_voltage < 0)
    {
        printstr("ERROR: NEGATIVE HOLD BRAKE VOLTAGE");
        return;
    }

    if(period_start_brake < 0)
    {
        printstr("ERROR: NEGATIVE PERIOD START BRAKE SETTINGS!");
        return;
    }

    MotorcontrolConfig motorcontrol_config = i_torque_control.get_config();

    if(motorcontrol_config.ifm_tile_usec==250)
    {
        duty_min = 1500;
        duty_max = 13000;
        duty_divider = 16384;
    }
    else if(motorcontrol_config.ifm_tile_usec==100)
    {
        duty_min = 600;
        duty_max = 7000;
        duty_divider = 8192;
    }
    else if (motorcontrol_config.ifm_tile_usec!=100 && motorcontrol_config.ifm_tile_usec!=250)
    {
        error = 1;
    }

    duty_start_brake    = (duty_divider * pull_brake_voltage)/(1000*dc_bus_voltage);
    if(duty_start_brake < duty_min) duty_start_brake = duty_min;
    if(duty_start_brake > duty_max) duty_start_brake = duty_max;

    duty_maintain_brake = (duty_divider * hold_brake_voltage)/(1000*dc_bus_voltage);
    if(duty_maintain_brake < duty_min) duty_maintain_brake = duty_min;
    if(duty_maintain_brake > duty_max) duty_maintain_brake = duty_max;

    period_start_brake  = (pull_brake_time * 1000)/(motorcontrol_config.ifm_tile_usec);

    i_update_brake.update_brake_control_data(duty_start_brake, duty_maintain_brake, period_start_brake);

    // send voltage to lifting motor
/*    int limit_switch
    if

    printf("wheel folded \n");*/
}

// This function listens to the signal sent by the button and uses the button interface to tell the behavior function whenever the button is pressed
void button_listener(client interface button_interface i_button_interface)
{
    int button_value = 0;
    timer t;
    unsigned  time;
    unsigned  time_2;
    t :> time;
    while(1)
    {
        select
        {
            case button when pinsneq(button_value) :> button_value:
                if(button_value == 1)
                {
                    t :> time;
                    printf("button = 1 \n");

                }
                else
                {
                    t :> time_2;
                    if(time_2-time>200000000)
                    {
                        i_button_interface.button_input(1);
                        printf("button_input(1) \n");
                    }
                    else
                    {
                        i_button_interface.button_input(0);
                        printf("button_input(0) \n");
                    }
                    t :> time;
                }
                break;
        }
    }
}

// This function is used to set one of the LED registers (0 -> 1002, 1 -> 1003 and 2 -> 1004) to a certain value.
int set_led_register(client interface rs485_interface h,int register_number, char * register_value, server interface LED_feedback i_LED_feedback)
{
    char message[] = ":0A0603E0000000RL";
    switch(register_number)
    {
    case 0:
        message[8] = 'A';
        break;
    case 1:
        message[8] = 'B';
        break;
    case 2:
        message[8] = 'C';
        break;
    }
    for(int i=0;i<4;i++)
    {
        message[9+i] = register_value[i];
    }
    int LRC_0;
    int LRC_1;
    {LRC_0,LRC_1} = compute_LRC(message);
    message[13] = int_to_char_ascii(LRC_1);
    message[14] = int_to_char_ascii(LRC_0);
    int number_of_characters = 17;
    int number_of_bits=1 + number_of_characters*11 ;
    int buffer[1000] = {};
    generate_message(message,number_of_characters,(int *)buffer);
    int number_of_try = 0;
    timer tmr;
    unsigned  time;
    h.send_message(buffer,number_of_bits);
    tmr :> time;
    time += (int)100000000/40;
    while(number_of_try<MAX_NUMBER_OF_ATTEMPTS)
    {
        number_of_try++;
        select
        {
            case i_LED_feedback.success():
                delay_milliseconds(10);
                return(1);
                break;
            case tmr  when  timerafter(time) :> int  now:
                h.send_message(buffer,number_of_bits);
                tmr :> time;
                time += (int)100000000/40;
                break;
        }
    }
    return(0);
}


// This function computes the value that the LED registers should have in order to have a certain behavior (color, blink...)
void compute_led_registers(int sys_color, int sys_blink, int batt_color, int batt_blink, int buzzer, int buzzer_blink, char * register_1, char * register_2, char * register_3)
{
    int integer_value_1 = 0;
    int integer_value_2 = 0;
    int integer_value_3 = 0;
    if(buzzer==1)
    {
        integer_value_1 = integer_value_1+128;
        if(buzzer_blink==1)
        {
            integer_value_1 = integer_value_1+32768;
            integer_value_3 = integer_value_3+10+10*256;
        }
    }
    integer_value_1 = integer_value_1+sys_color;
    integer_value_1 = integer_value_1+8*batt_color;
    if(sys_blink == 1 || batt_blink == 1)
    {
        integer_value_2 = integer_value_2+10+10*256;
    }
    if(sys_blink == 1)
    {
        integer_value_1 = integer_value_1+256*sys_color;
    }
    if(batt_blink == 1)
    {
        integer_value_1 = integer_value_1+256*8*batt_color;
    }

    ///////

    register_1[0] = (char)int_to_char_hex((int)integer_value_1/4096);
    while(integer_value_1>=4096)
    {
        integer_value_1 = integer_value_1-4096;
    }
    register_1[1] = (char)int_to_char_hex((int)integer_value_1/256);
    while(integer_value_1>=256)
    {
        integer_value_1 = integer_value_1-256;
    }
    register_1[2] = (char)int_to_char_hex((int)integer_value_1/16);
    while(integer_value_1>=16)
    {
        integer_value_1 = integer_value_1-16;
    }
    register_1[3] = (char)int_to_char_hex((int)integer_value_1);

    ///////

    register_2[0] = (char)int_to_char_hex((int)integer_value_2/4096);
    while(integer_value_2>=4096)
    {
        integer_value_2 = integer_value_2-4096;
    }
    register_2[1] = (char)int_to_char_hex((int)integer_value_2/256);
    while(integer_value_2>=256)
    {
        integer_value_2 = integer_value_2-256;
    }
    register_2[2] = (char)int_to_char_hex((int)integer_value_2/16);
    while(integer_value_2>=16)
    {
        integer_value_2 = integer_value_2-16;
    }
    register_2[3] = (char)int_to_char_hex((int)integer_value_2);

    ///////

    register_3[0] = (char)int_to_char_hex((int)integer_value_3/4096);
    while(integer_value_3>=4096)
    {
        integer_value_3 = integer_value_3-4096;
    }
    register_3[1] = (char)int_to_char_hex((int)integer_value_3/256);
    while(integer_value_3>=256)
    {
        integer_value_3 = integer_value_3-256;
    }
    register_3[2] = (char)int_to_char_hex((int)integer_value_3/16);
    while(integer_value_3>=16)
    {
        integer_value_3 = integer_value_3-16;
    }
    register_3[3] = (char)int_to_char_hex((int)integer_value_3);
}

// This function is used to request the value of the switch register in the handle and wait for a response.
// If after a certain amount of attempts the handle has not responded, an error is raised.
int get_switch_position(client interface rs485_interface h,server interface switch_feedback i_switch_feedback)
{
    char message[] = ":0A0403E8000106RL";
    int number_of_characters = 17;
    int number_of_bits= 1 + number_of_characters*11 ;
    int buffer[1000] ;
    generate_message(message,number_of_characters,(int *)buffer);
    h.send_message(buffer,number_of_bits);
    int number_of_try = 0;
    timer tmr;
    unsigned  time;
    tmr :> time;
    time += (int)100000000/40;
    while(number_of_try<MAX_NUMBER_OF_ATTEMPTS)
    {
        number_of_try++;
        select
        {
            case i_switch_feedback.switch_input(int integer):
                return(integer);
            case tmr  when  timerafter(time) :> int  now:
                h.send_message(buffer,number_of_bits);
                tmr :> time;
                time += (int)100000000/40;
                break;
        }
    }
    return(1234);
}

// TODO : Implement this
int get_battery_level()
{
    return(70);
}

// TODO : Implement this
// This function updates the torque output from the BLDC travel motor based on the switch value
// The higher the absolute switch value, the higher the torque
// Positive switch_value is forward, negative is backward
void update_torque(int switch_value, client interface TorqueControlInterface i_torque_control)
{

    int torque_factor = 1500/100;
    int bldc_torque = switch_value*torque_factor;
    i_torque_control.set_torque_control_enabled();
    i_torque_control.set_torque(bldc_torque);

    printf("torque set to : %d mNm \n", bldc_torque);

}

// This function handles everything.
// TODO : Finish this (Button , emergency stop, implement pwm alerts if they exist)
void behavior(client interface rs485_interface h, server interface switch_feedback i_switch_feedback, server interface button_interface j, server interface LED_feedback k, client interface TorqueControlInterface i_torque_control, client interface UpdateBrake i_update_brake)
{
    MotorcontrolConfig  motorcontrol_config;

    char * led_register_1 = "0000";
    char * led_register_2 = "0000";
    char * led_register_3 = "0000";

    int system_led_color = 1;
    int system_led_blink = 0;
    int battery_led_color = 0;
    int battery_led_blink = 0;
    int buzzer_active = 0;
    int buzzer_blink = 0;

    int battery_level = 0;

    int switch_value = 0;

    int state = 0;
        /*
         *     0 -> OFF
         *     1 -> IDLE
         *     2 -> Running
         *     3 -> Error
         */

    int error_type = 0;
    /*
     *   0 -> Unknown
     *   1 -> No response from handle
     */

    int success_1 = 1;
    int success_2 = 1;
    int success_3 = 1;

    int handle_connected = 0;

    while(handle_connected == 0)
    {
        compute_led_registers(1,0,0,0,0,0,led_register_1,led_register_2,led_register_3);
        success_1 = set_led_register(h,0,led_register_1,k);
        success_2 = set_led_register(h,1,led_register_2,k);
        success_3 = set_led_register(h,2,led_register_3,k);

        if(success_1*success_2*success_3==0)
        {
            printf("PLEASE CONNECT THE HANDLE \n");
        }
        else
        {
            handle_connected = 1;
            printf("Handle detected \n");
        }
        delay_milliseconds(500);
    }

    timer timer_1;
    timer timer_2;
    unsigned  time_1;
    unsigned  time_2;
    timer_1 :> time_1;
    timer_2 :> time_2;

    motorcontrol_config = i_torque_control.get_config();
    int app_tile_usec = 100;
    int dc_bus_voltage = motorcontrol_config.dc_bus_voltage; // volts
    int pull_brake_voltage = 16000; // milivolts
    int hold_brake_voltage = 1000; // millivolts
    int pull_brake_time = 2000; // milli-seconds

    while(1)
    {
        switch(state)
        {
        case 0:
            select
            {
                case j.button_input(int integer):
                    if(integer==1)
                    {
                        state = 1;
                        battery_level = get_battery_level();
                        if(battery_level<50)
                        {
                            battery_led_color = 1;
                            battery_led_blink = 1;
                            buzzer_active = 1;
                            buzzer_blink = 1;
                        }
                        else if(battery_level<60)
                        {
                            battery_led_color = 1;
                        }
                        else if(battery_level<80)
                        {
                            battery_led_color = 3;
                        }
                        else
                        {
                            battery_led_color = 2;
                        }
                        system_led_color = 2;
                        compute_led_registers(system_led_color,system_led_blink,battery_led_color,battery_led_blink,buzzer_active,buzzer_blink,led_register_1,led_register_2,led_register_3);
                        success_1 = set_led_register(h,0,led_register_1,k);
                        success_2 = set_led_register(h,1,led_register_2,k);
                        success_3 = set_led_register(h,2,led_register_3,k);
                        if(success_1*success_2*success_3==0)
                        {
                            printf("ERROR : NO RESPONSE FROM HANDLE \n");
                            error_type = 1;
                            i_torque_control.set_torque(0);
                            state = 3;
                        }
                        timer_1 :> time_1;
                        timer_2 :> time_2;
                    }
                    break;

            }
            break;
        case 1 :
            int unfold_request = 0;
            int update_battery_request = 0;
            select
            {
                case j.button_input(int integer):
                    if(integer==0)
                    {
                        unfold_request = 1;
                    }
                    break;
                case timer_1  when  timerafter(time_1) :> int  now:
                    time_1 += (int)100000000/10;
                    int switch_pos = get_switch_position(h,i_switch_feedback);
                    if(switch_pos == 1234)
                    {
                        printf("ERROR : NO RESPONSE FROM HANDLE \n");
                        error_type = 1;
                        i_torque_control.set_torque(0);
                        state = 3;
                    }
                    else
                    {
                        switch_value = switch_pos;
                        printf("switch position : %d \n",switch_value);
                        //i_torque_control.set_torque(switch_value * 15);
                        //printf("set torque to : %d", switch_value * 15);
                        if (switch_value > 95)
                        {
                            unfold_request = 1;
                        }
                    }
                    break;
                case timer_2 when timerafter(time_2) :> int  now:
                    time_2 += (int)1000000000; // every 10 seconds
                    battery_level = get_battery_level();
                    if(battery_level<50)
                    {
                        battery_led_color = 1;
                        buzzer_active = 1;
                        buzzer_blink = 1;
                    }
                    else if(battery_level<60)
                    {
                        battery_led_color = 1;
                    }
                    else if(battery_level<80)
                    {
                        battery_led_color = 3;
                    }
                    else
                    {
                        battery_led_color = 2;
                    }
                    system_led_color = 2;
                    compute_led_registers(system_led_color,system_led_blink,battery_led_color,battery_led_blink,buzzer_active,buzzer_blink,led_register_1,led_register_2,led_register_3);
                    success_1 = set_led_register(h,0,led_register_1,k);
                    success_2 = set_led_register(h,1,led_register_2,k);
                    success_3 = set_led_register(h,2,led_register_3,k);
                    if(success_1*success_2*success_3==0)
                    {
                        printf("ERROR : NO RESPONSE FROM HANDLE \n");
                        error_type = 1;
                        i_torque_control.set_torque(0);
                        state = 3;
                    }
                    break;
            }
            if(unfold_request == 1)
            {
                unfold_wheel(app_tile_usec,
                        dc_bus_voltage,
                        pull_brake_voltage,
                        hold_brake_voltage,
                        pull_brake_time,
                        i_torque_control, i_update_brake);
                system_led_color = 2;
                system_led_blink = 1;
                compute_led_registers(system_led_color,system_led_blink,battery_led_color,battery_led_blink,buzzer_active,buzzer_blink,led_register_1,led_register_2,led_register_3);
                set_led_register(h,0,led_register_1,k);
                set_led_register(h,1,led_register_2,k);
                set_led_register(h,2,led_register_3,k);
                state = 2;
                printf("state is now : %d \n", state);
            }
            break;
        case 2 :
            int button_long = 0;
            int button_short = 0;
            int switch_value_updated = 0;
            i_torque_control.set_torque_control_enabled();
            select
            {
            case j.button_input(int integer):
                 if(integer==1)
                 {
                     button_long = 1;
                 }
                 else
                 {
                     button_short = 1;
                 }
                 break;
            case timer_1  when  timerafter(time_1) :> int  now:
                int switch_pos = get_switch_position(h,i_switch_feedback);
                timer_1 :> time_1;
                time_1 += (int)100000000/100; // wait 10 ms before requesting the position of the switch again
                if(switch_pos == 1234)
                {
                    printf("ERROR : NO RESPONSE FROM HANDLE \n");
                    error_type = 1;
                    i_torque_control.set_torque(0);
                    state = 3;
                }
                else
                {
                    switch_value = switch_pos;
                    switch_value_updated = 1;
                 }
                 break;
            case timer_2 when timerafter(time_2) :> int  now:
                 time_2 += (int)1000000000;
                 battery_level = get_battery_level();
                 if(battery_level<50)
                 {
                       battery_led_color = 1;
                       buzzer_active = 1;
                       buzzer_blink = 1;
                 }
                 else if(battery_level<60)
                 {
                     battery_led_color = 1;
                 }
                 else if(battery_level<80)
                 {
                     battery_led_color = 3;
                 }
                 else
                 {
                     battery_led_color = 2;
                 }
                 compute_led_registers(system_led_color,system_led_blink,battery_led_color,battery_led_blink,buzzer_active,buzzer_blink,led_register_1,led_register_2,led_register_3);
                 success_1 = set_led_register(h,0,led_register_1,k);
                 success_2 = set_led_register(h,1,led_register_2,k);
                 success_3 = set_led_register(h,2,led_register_3,k);
                 if(success_1*success_2*success_3==0)
                 {
                       printf("ERROR : NO RESPONSE FROM HANDLE \n");
                       error_type = 1;
                       i_torque_control.set_torque(0);
                       state = 3;
                 }
                 break;
            }
            if(switch_value_updated==1)
            {
                printf("set torque to : %d mNm\n", switch_value * 15);
                i_torque_control.set_torque(switch_value * 15);
            }
            if(button_short==1)
            {
                switch_value = 0;
                i_torque_control.set_torque(switch_value * 15);
                fold_wheel(app_tile_usec,
                        dc_bus_voltage,
                        pull_brake_voltage,
                        hold_brake_voltage,
                        pull_brake_time,
                        i_torque_control, i_update_brake);
                state = 1;
                timer_1 :> time_1;
                timer_2 :> time_2;
            }
            if(button_long==1)
            {
                switch_value = 0;
                i_torque_control.set_torque(switch_value * 15);
                //motorcontrol_config = i_torque_control.get_config();
                fold_wheel(app_tile_usec,
                        dc_bus_voltage,
                        pull_brake_voltage,
                        hold_brake_voltage,
                        pull_brake_time,
                        i_torque_control, i_update_brake);
                state = 0;
            }
            break;
        case 3:
            if(error_type == 1)
            {
                battery_level = get_battery_level();
                if(battery_level<50)
                {
                    battery_led_color = 1;
                    battery_led_blink = 1;
                    buzzer_active = 1;
                    buzzer_blink = 1;
                }
                else if(battery_level<60)
                {
                    battery_led_color = 1;
                }
                else if(battery_level<80)
                {
                    battery_led_color = 3;
                }
                else
                {
                    battery_led_color = 2;
                }
                system_led_color = 2;
                system_led_blink = 0;
                compute_led_registers(system_led_color,system_led_blink,battery_led_color,battery_led_blink,buzzer_active,buzzer_blink,led_register_1,led_register_2,led_register_3);
                success_1 = set_led_register(h,0,led_register_1,k);
                success_2 = set_led_register(h,1,led_register_2,k);
                success_3 = set_led_register(h,2,led_register_3,k);
                if(success_1*success_2*success_3==1)
                {
                    printf("Handle detected \n");
                    fold_wheel(app_tile_usec,
                            dc_bus_voltage,
                            pull_brake_voltage,
                            hold_brake_voltage,
                            pull_brake_time,
                            i_torque_control, i_update_brake);
                    error_type = 0;
                    state = 1;
                }
            }
            break;
        }
    }

}

int  main() {

    // Motor control interfaces
    interface WatchdogInterface i_watchdog[2];
    interface UpdateBrake i_update_brake;
    interface ADCInterface i_adc[2];
    interface TorqueControlInterface i_torque_control[2];
    interface shared_memory_interface i_shared_memory[3];
    interface PositionFeedbackInterface i_position_feedback_1[3];
    interface PositionFeedbackInterface i_position_feedback_2[3];
    interface UpdatePWM i_update_pwm;

    par
    {
      on tile[COM_TILE]:
      {
          interface rs485_interface i_rs485_interface;
          interface switch_feedback j;
          interface button_interface k;
          interface LED_feedback n;
          par
          {
              button_listener(k);
              behavior(i_rs485_interface,j,k,n, i_torque_control[0], i_update_brake);
              handle_rs485(19200,i_rs485_interface,j,n);
          }
      }
      on tile[IFM_TILE]:
      {
          //interface pwm_interface l;
          //generate_pwm(l);

          par
          {
                          /* PWM Service */
                          {
                              pwm_config(pwm_ports);

                              if (!isnull(fet_driver_ports.p_esf_rst_pwml_pwmh) && !isnull(fet_driver_ports.p_coast))
                                  predriver(fet_driver_ports);

                              //pwm_check(pwm_ports);//checks if pulses can be generated on pwm ports or not
                              pwm_service_task(MOTOR_ID, pwm_ports, i_update_pwm,
                                      i_update_brake, IFM_TILE_USEC);

                          }

                          /* ADC Service */
                          {
                              adc_service(adc_ports, i_adc /*ADCInterface*/, i_watchdog[1], IFM_TILE_USEC, SINGLE_ENDED);
                          }

                          /* Watchdog Service */
                          {
                              watchdog_service(wd_ports, i_watchdog, IFM_TILE_USEC);
                          }

                          /* Motor Control Service */
                          {

                              MotorcontrolConfig motorcontrol_config;

                              motorcontrol_config.dc_bus_voltage =  DC_BUS_VOLTAGE;
                              motorcontrol_config.phases_inverted = MOTOR_PHASES_CONFIGURATION;
                              motorcontrol_config.torque_P_gain =  TORQUE_Kp;
                              motorcontrol_config.torque_I_gain =  TORQUE_Ki;
                              motorcontrol_config.torque_D_gain =  TORQUE_Kd;
                              motorcontrol_config.pole_pairs =  MOTOR_POLE_PAIRS;
                              motorcontrol_config.commutation_sensor=SENSOR_1_TYPE;
                              motorcontrol_config.commutation_angle_offset=COMMUTATION_ANGLE_OFFSET;
                              motorcontrol_config.hall_state_angle[0]=HALL_STATE_1_ANGLE;
                              motorcontrol_config.hall_state_angle[1]=HALL_STATE_2_ANGLE;
                              motorcontrol_config.hall_state_angle[2]=HALL_STATE_3_ANGLE;
                              motorcontrol_config.hall_state_angle[3]=HALL_STATE_4_ANGLE;
                              motorcontrol_config.hall_state_angle[4]=HALL_STATE_5_ANGLE;
                              motorcontrol_config.hall_state_angle[5]=HALL_STATE_6_ANGLE;
                              motorcontrol_config.max_torque =  MOTOR_MAXIMUM_TORQUE;
                              motorcontrol_config.phase_resistance =  MOTOR_PHASE_RESISTANCE;
                              motorcontrol_config.phase_inductance =  MOTOR_PHASE_INDUCTANCE;
                              motorcontrol_config.torque_constant =  MOTOR_TORQUE_CONSTANT;
                              motorcontrol_config.current_ratio =  CURRENT_RATIO;
                              motorcontrol_config.voltage_ratio =  VOLTAGE_RATIO;
                              motorcontrol_config.temperature_ratio =  TEMPERATURE_RATIO;
                              motorcontrol_config.rated_current =  MOTOR_RATED_CURRENT;
                              motorcontrol_config.rated_torque  =  MOTOR_RATED_TORQUE;
                              motorcontrol_config.percent_offset_torque =  APPLIED_TUNING_TORQUE_PERCENT;
                              motorcontrol_config.protection_limit_over_current =  PROTECTION_MAXIMUM_CURRENT;
                              motorcontrol_config.protection_limit_over_voltage =  PROTECTION_MAXIMUM_VOLTAGE;
                              motorcontrol_config.protection_limit_under_voltage = PROTECTION_MINIMUM_VOLTAGE;
                              motorcontrol_config.protection_limit_over_temperature = TEMP_BOARD_MAX;

                              torque_control_service(motorcontrol_config, i_adc[0], i_shared_memory[2],
                                      i_watchdog[0], i_torque_control, i_update_pwm, IFM_TILE_USEC);
                          }

                          /* Shared memory Service */
                          [[distribute]] shared_memory_service(i_shared_memory, 3);

                          /* Position feedback service */
                          {
                              PositionFeedbackConfig position_feedback_config;
                              position_feedback_config.sensor_type = SENSOR_1_TYPE;
                              position_feedback_config.resolution  = SENSOR_1_RESOLUTION;
                              position_feedback_config.polarity    = SENSOR_1_POLARITY;
                              position_feedback_config.velocity_compute_period = SENSOR_1_VELOCITY_COMPUTE_PERIOD;
                              position_feedback_config.pole_pairs  = MOTOR_POLE_PAIRS;
                              position_feedback_config.ifm_usec    = IFM_TILE_USEC;
                              position_feedback_config.max_ticks   = SENSOR_MAX_TICKS;
                              position_feedback_config.offset      = HOME_OFFSET;
                              position_feedback_config.sensor_function = SENSOR_1_FUNCTION;

                              position_feedback_config.biss_config.multiturn_resolution = BISS_MULTITURN_RESOLUTION;
                              position_feedback_config.biss_config.filling_bits = BISS_FILLING_BITS;
                              position_feedback_config.biss_config.crc_poly = BISS_CRC_POLY;
                              position_feedback_config.biss_config.clock_frequency = BISS_CLOCK_FREQUENCY;
                              position_feedback_config.biss_config.timeout = BISS_TIMEOUT;
                              position_feedback_config.biss_config.busy = BISS_BUSY;
                              position_feedback_config.biss_config.clock_port_config = BISS_CLOCK_PORT;
                              position_feedback_config.biss_config.data_port_number = BISS_DATA_PORT_NUMBER;

                              position_feedback_config.rem_16mt_config.filter = REM_16MT_FILTER;

                              position_feedback_config.rem_14_config.hysteresis              = REM_14_SENSOR_HYSTERESIS;
                              position_feedback_config.rem_14_config.noise_settings          = REM_14_SENSOR_NOISE_SETTINGS;
                              position_feedback_config.rem_14_config.dyn_angle_error_comp    = REM_14_DYN_ANGLE_ERROR_COMPENSATION;
                              position_feedback_config.rem_14_config.abi_resolution_settings = REM_14_ABI_RESOLUTION_SETTINGS;

                              position_feedback_config.qei_config.number_of_channels = QEI_SENSOR_NUMBER_OF_CHANNELS;
                              position_feedback_config.qei_config.signal_type        = QEI_SENSOR_SIGNAL_TYPE;
                              position_feedback_config.qei_config.port_number        = QEI_SENSOR_PORT_NUMBER;

                              position_feedback_config.hall_config.port_number = HALL_SENSOR_PORT_NUMBER;

                              position_feedback_config.gpio_config[0] = GPIO_OFF;
                              position_feedback_config.gpio_config[1] = GPIO_OFF;
                              position_feedback_config.gpio_config[2] = GPIO_OFF;
                              position_feedback_config.gpio_config[3] = GPIO_OFF;

                              //setting second sensor
                              PositionFeedbackConfig position_feedback_config_2 = position_feedback_config;
                              position_feedback_config_2.sensor_type = 0;
                              if (SENSOR_2_FUNCTION != SENSOR_FUNCTION_DISABLED) //enable second sensor
                              {
                                  position_feedback_config_2.sensor_type = SENSOR_2_TYPE;
                                  position_feedback_config_2.polarity    = SENSOR_2_POLARITY;
                                  position_feedback_config_2.resolution  = SENSOR_2_RESOLUTION;
                                  position_feedback_config_2.velocity_compute_period = SENSOR_2_VELOCITY_COMPUTE_PERIOD;
                                  position_feedback_config_2.sensor_function = SENSOR_2_FUNCTION;
                              }

                              position_feedback_service(qei_hall_port_1, qei_hall_port_2, hall_enc_select_port, spi_ports, gpio_port_0, gpio_port_1, gpio_port_2, gpio_port_3,
                                      position_feedback_config, i_shared_memory[0], i_position_feedback_1,
                                      position_feedback_config_2, i_shared_memory[1], i_position_feedback_2);

                          }
          }
      }
  }
  return  0;
}
