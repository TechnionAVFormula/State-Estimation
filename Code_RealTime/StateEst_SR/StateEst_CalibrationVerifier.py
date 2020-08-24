import time


class CalibrationVerifier():
    def __init__(self , **kwargs):
        """ CalibrationVerifier: makes sure that our module doesn't fire false information too quickly
        when we actually know nothing about the world
        
        Kwargs:

            minTime {float} -- How much time [seconds] to be asleep before we send information to Control.
            minIterations {int} -- How many iterations to be asleep before we send information to Control.
            minCones {int} -- How many Cones should we see on either side before we send information to Control.
        """
        self._is_awake = False

        self.InitTime = time.monotonic()
        #Default values for thresholds:
        self.minTime = 0  #[sec]
        self.minCones = 0 #[number of detected cones]
        self.minIterations = 0 #[number passed iteration]
        #memory of last given data:
        self.lastTime = 0
        self.lastRightConesNum = 0
        self.lastLeftConesNum  = 0
        self.lastIteration = 0

        # Possible Inputs:
        for key, value in kwargs.items(): 
            if key == 'minTime':                                                    
                self.minTime = value               
            if key == 'minCones':
                self.minCones = value
            if key == 'minIterations':
                self.minIterations = value

    def checkCalibrated(self, **kwargs):
        """ Check Sleeper if now we can send data to Control. Tell him everything you know. 
        You don't have to remember for him, you can give him partial information each time.
        
        Kwargs:

            numLeftCones  {int} -- How many Cones have we seen on the left.
            numRightCones {int} -- How many Cones have we seen on the right.

        """        
        # if already awake, don't check again, no matter what: (to save computation time)
        if self._is_awake:
            return True

        # Possible Inputs. Update only if this info was given:
        for key, value in kwargs.items(): 
            if key == 'numLeftCones':
                self.lastLeftConesNum  = value
            if key == 'numRightCones':
                self.lastRightConesNum = value

        self.lastIteration += 1
        self.lastTime = time.monotonic()

        # now check if awake:
        shouldWakeup =( ( self.lastLeftConesNum >= self.minCones ) and ( self.lastRightConesNum >= self.minCones ) and
                        ( self.lastIteration >= self.minIterations ) and ( self.lastTime - self.InitTime >= self.minTime ) )
        if shouldWakeup:   
            self._is_awake = True # So next time we won't need to check this again.
            return True
        else:
            return False
        



if __name__ == "__main__":
    vc = CalibrationVerifier( minTime = 0 , minCones = 4 , minIterations = 4 )
    res = vc.checkCalibrated(numRightCones=2 , numLeftCones=4 )
    print(f"Last results was =-={str(res):<5}=-=")
    res = vc.checkCalibrated(numRightCones=4 , numLeftCones=4 )
    print(f"Last results was =-={str(res):<5}=-=")
    res = vc.checkCalibrated(numRightCones=4 , numLeftCones=4 )
    print(f"Last results was =-={str(res):<5}=-=")
    res = vc.checkCalibrated(numRightCones=4 , numLeftCones=4 )
    print(f"Last results was =-={str(res):<5}=-=")
    res = vc.checkCalibrated(numRightCones=1 , numLeftCones=2 ) #re-check
    print(f"Last results was =-={str(res):<5}=-=")
