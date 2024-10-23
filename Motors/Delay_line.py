
from pytrinamic.features.linear_ramp import LinearRamp
from pytrinamic.connections import ConnectionManager
import time
import sys
import os
import logging
from pytrinamic.features import StallGuard2Module,CoolStepModule
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from Motors.trinamic_controller import TMCM3212

# Setup logging configuration to write to delaylinelog.txt
logging.basicConfig(filename="delaylinelog.txt", 
                    level=logging.INFO, 
                    format="%(asctime)s - %(message)s")


class DelayLine(TMCM3212):
    def __init__(self,connection,module_id=1,ap_index_bitwidth=8,step_angle=1.8, lead_pitch=5.08):
        super().__init__(connection,module_id,ap_index_bitwidth)
        self.connection = connection
        self.ap_index_bit_width = ap_index_bitwidth
        self.module_id = module_id
        self.module=TMCM3212(self.connection)
        self.motor=self.module.motors[0]

        #Calculate the steps in each revolution
        self.steps_rev = int((360 / step_angle) * self.motor.MR.microstep_resolution_256_microsteps)
        
        self.lead_pitch=lead_pitch

        #Assigning the minimum and maximum positions of the motor
        self.min_position = int((-200 * self.steps_rev) / self.lead_pitch)
        self.max_position = int((200 * self.steps_rev) / self.lead_pitch)


        # Set motor parameters
        logging.info('Setting motor parameters')
        self.motor.drive_settings.max_current=8#Set to 0.40A
        self.motor.drive_settings.standby_current=0 # Typically half of max current
        self.motor.drive_settings.boost_current=0  # Set if additional current is needed
        self.motor.drive_settings.microstep_resolution = self.motor.ENUM.microstep_resolution_256_microsteps
        logging.info(self.motor.drive_settings)

        self.motor.set_axis_parameter(self.motor.AP.RunCurrent,8)
        logging.info(f'Run current: {self.motor.get_axis_parameter(self.motor.AP.RunCurrent)}')

        self.motor.actual_position=0

        #Setting the velocity and acceleration of the axis
        self.motor.set_axis_parameter(self.motor.AP.MaxAcceleration,30000)
        self.motor.set_axis_parameter(self.motor.AP.MaxVelocity,30000)
        logging.info(self.motor.linear_ramp)

        #Set the freewheeling mode 
        self.motor.set_axis_parameter(self.motor.AP.FreewheelingMode, value=3)

        #PWM data
        self.motor.set_axis_parameter(self.motor.AP.PWMGrad, value=1)
        self.motor.set_axis_parameter(self.motor.AP.PWMAmplitude, value=64)
        self.motor.set_axis_parameter(self.motor.AP.PWMAutoscale, value=1)
        self.motor.set_axis_parameter(self.motor.AP.PWMThresholdSpeed, value=51200)

        logging.info(f'PWM Mode: {self.motor.get_axis_parameter(self.motor.AP.PWMMode)}')
        logging.info(f'PWM Frequency: {self.motor.get_axis_parameter(self.motor.AP.PWMFrequency)}')
        logging.info(f'PWM Threshold speed: {self.motor.get_axis_parameter(self.motor.AP.PWMThresholdSpeed)}')


    def rotate(self, axis,velocity):
        self.connection.rotate(axis, velocity, self.module_id)

    def move_to(self, axis, position, velocity=None):
        steps=int((position*self.steps_rev)/self.lead_pitch)
        if steps < self.min_position:
            steps = self.min_position
            logging.info("Beyond minimum position, can't move.")
        elif steps > self.max_position:
            steps = self.max_position
            logging.info("Beyond maximum position reached, can't move.")
    
        if velocity:
            self.motors[axis].linear_ramp.max_velocity = velocity
        logging.info(f'Moving to position {position} with steps {steps}')
        self.connection.move_to(axis, steps, self.module_id)

    def move_by(self, axis, difference, velocity=None):
        target_position=self.get_position() + difference
        if velocity:
            self.motor.linear_ramp.max_velocity = velocity
        logging.info(f'Moving by difference {difference}, target position {target_position}')
        self.move_to(axis, target_position, self.module_id)

    def stop(self, axis):
        logging.info(f'Stopping axis {axis}')
        self.connection.stop(axis, self.module_id)
    
    def get_position(self):
        return self.motor.get_axis_parameter(self.motor.AP.ActualPosition)
       
    def is_position_reached(self):
        return self.motor.get_axis_parameter(self.motor.AP.PositionReachedFlag)
    
    def go_to_home_position(self):
        time_out=20
        start_time=time.time()
        logging('Starting homing procedure')
        
        # Set reference speed
        self.motor.set_axis_parameter(self.motor.AP.ReferenceSearchSpeed, 10000)
        self.motor.set_axis_parameter(self.motor.AP.RefSwitchSpeed, 500)

        home_state=self.motor.get_axis_parameter(self.motor.AP.HomeSwitch)
        logging.info(f'Initial Home state: {home_state}')

        try:
            initial_status = self.get_reference_search_status(motor=0)
            logging.info(f'Reference search status initially: {initial_status}')
            
            # Start reference search
            if home_state==0:
                self.start_reference_search(motor=0, mode=8)
                logging.info('Starting reference search')

                while True:
                    current_home_state = self.motor.get_axis_parameter(self.motor.AP.HomeSwitch)
                    logging.info(f'Current home state: {current_home_state}')

                    # Check if home state is 1 (meaning home is reached)
                    if current_home_state == 1:
                        logging.info('Home position reached')
                        self.stop_reference_search(motor=0)
                        self.motor.set_axis_parameter(self.motor.AP.ActualPosition, 0)
                        logging.info(f'Motor actual position: {self.motor.actual_position}')
                        break

                    if time.time() - start_time > time_out:
                        logging.warning('Homing procedure timed out')
                        self.stop(0)
                        break

                    time.sleep(0.2)
                
        
            
            elif home_state==1:
                self.start_reference_search(motor=0, mode=7)
                logging.info('Starting reference search')

                while True:
                    current_home_state = self.motor.get_axis_parameter(self.motor.AP.HomeSwitch)
                    logging.info(f'Current home state: {current_home_state}')

                    # Check if home state is 1 (meaning home is reached)
                    if current_home_state == 0:
                        logging.info('Home position reached')
                        self.stop_reference_search(motor=0)
                        self.motor.set_axis_parameter(self.motor.AP.ActualPosition, 0)
                        logging.info(f'Motor actual position: {self.motor.actual_position}')
                        break

                    if time.time() - start_time > time_out:
                        logging.warning('Homing procedure timed out')
                        self.stop(0)
                        break

                    time.sleep(0.2)
             
            
            logging.info('Homing procedure completed')
        
        except Exception as e:
            logging.info(f"An error occurred: {e}")
            # Attempt to stop the motor gracefully
            try:
                self.stop_reference_search(motor=0)  # Stop reference search
                logging.info("Stopped reference search.")
            except Exception as stop_error:
                logging.info(f"Failed to stop the motor gracefully: {stop_error}")