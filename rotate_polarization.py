import time
import sys
import os
import logging
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from Motors.Polarization import PolarizationPaddler
from pytrinamic.connections import ConnectionManager


# Setup logging configuration to write to delaylinelog.txt
logging.basicConfig(filename="rotate_polarization_log.txt", 
                    level=logging.INFO, 
                    format="%(asctime)s - %(message)s")

def main():
    try:
        connection_manager=ConnectionManager()
        with connection_manager.connect() as my_interface:
            logging.info('Connection established')
            pol = PolarizationPaddler(my_interface)
            # target_position=30
            # time_out=10
            # start_time=time.time()
            #pol.move_to(axis=1,position=30)
            #pol.go_to_home_position()
            # pol.move_to(axis=1,position=target_position)

            # while not pol.is_position_reached():
            #     current_position=pol.get_position()
            #     actual_velocity = pol.motor.get_axis_parameter(pol.motor.AP.ActualVelocity)
            #     logging.info(f'Current position: {current_position}, velocity: {actual_velocity}')
            #     time.sleep(0.2)
            # #logging.info(f'Velocity: {pol.motor.actual_velocity}')
            #     if time.time()-start_time > time_out:
            #         logging.info('Timeout reached')
            #         pol.stop(axis=1)
            #         break
            # else:
            #     logging.info('Target position reached') 
            #     pol.stop(axis=1)

            #pol.go_to_home_position()
            pol.move_to(axis=1,position=-180)
            time.sleep(3)
            pol.go_to_home_position()

        

    except Exception as e:
        logging.error(f"An error occurred: {e}")
        try:
            pol.rotate(0)  # Ensure the motor stops if an error occurs
            time.sleep(1)  # Allow some time for the motor to decelerate
        except:
            logging.error("Failed to stop the motor gracefully.")

logging.info('\n Ready')

if __name__== "__main__":
    main()
