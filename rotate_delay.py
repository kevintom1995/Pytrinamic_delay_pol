import time
import logging
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from Motors.Delay_line import DelayLine
from pytrinamic.connections import ConnectionManager


# Setup logging configuration to write to delaylinelog.txt
logging.basicConfig(filename="rotate_delayline_log.txt", 
                    level=logging.INFO, 
                    format="%(asctime)s - %(message)s")

def main():
    try:
        connection_manager=ConnectionManager()
        with connection_manager.connect() as my_interface:
            logging.info('Connection established')
            delay = DelayLine(my_interface)
            target_position=5
            time_out=10
            start_time=time.time()

            #Moving the delay line to home position
            delay.go_to_home_position()
            time.sleep(3)

            #Moving the delay line to a target position
            delay.move_to(axis=0,position=target_position)

            while not delay.is_position_reached():
                current_position=delay.get_position()
                actual_velocity = delay.motor.get_axis_parameter(delay.motor.AP.ActualVelocity)
                logging.info(f'Current position: {current_position}, velocity: {actual_velocity}')
                time.sleep(0.2)

                #Setting a time out
                if time.time()-start_time > time_out:
                    logging.info('Timeout reached')
                    delay.stop(axis=0)
                    break
            else:
                logging.info('Target position reached')
                delay.stop(axis=0)
            time.sleep(3)


    except Exception as e:
        logging.error(f"An error occurred: {e}")
        try:
            delay.stop(axis=0)  # Ensure the motor stops if an error occurs
            time.sleep(1)  # Allow some time for the motor to decelerate
        except:
            logging.error("Failed to stop the motor gracefully.")

logging.info('\n Ready')

if __name__ == "__main__":
    main()





