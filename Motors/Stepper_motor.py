from abc import ABC, abstractmethod


class Steppermotor(ABC):
    
    @abstractmethod
    def rotate(self,axis,velocity):
        """Rotates the motor with a specific velocity"""
        raise NotImplementedError

    @abstractmethod
    def move_to(self, position,velocity=None):
        """Move the motor to a specific position"""
        raise NotImplementedError
    
    @abstractmethod
    def move_by(self,axis,difference,velocity=None):
        """Moves the motor by a difference"""
        raise NotImplementedError

    @abstractmethod
    def stop(self):
        """Stop the motor"""
        raise NotImplementedError
    
    @abstractmethod
    def get_position(self,axis):
        """Gets the current position of the motor"""
        raise NotImplementedError
    
    @abstractmethod
    def get_position_reached(self):
        """Gets the position reached flag"""
        raise NotImplementedError
    
    @abstractmethod
    def go_to_home_position(self):
        """Moves the motor to the home position"""
        raise NotImplementedError

    
    
