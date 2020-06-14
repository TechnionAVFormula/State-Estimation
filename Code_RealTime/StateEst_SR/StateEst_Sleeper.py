import time


class SleeperManager():
    def __init__(self , **kwargs):
        """ Sleeper makes sure that our module doesn't five false information too quickly
        when we actually know nothing about the world
        
        Kwargs:

            minTime {float} -- How much time [seconds] to be asleep before we send information to Control.
            minIterations {int} -- How many iterations to be asleep before we send information to Control.
            minCones {int} -- How many Cones should we see before we send information to Control.
        """
        self._is_awake = False

        self.InitTime = time.monotonic()
        #Default values for thresholds:
        self.minTime = 0  #[sec]
        self.minCones = 0 #[number of detected cones]
        self.minIterations = 0 #[number passed iteration]
        #memory of last given data:
        self.lastTime = 0
        self.lastCones = 0
        self.lastIteration = 0

        # Possible Inputs:
        for key, value in kwargs.items(): 
            if key == 'minTime':                                                    
                self.minTime = value               
            if key == 'minCones':
                self.minCones = value
            if key == 'minIterations':
                self.minIterations = value

    def checkAwake(self, **kwargs):
        """ Check Sleeper if now we can send data to Control. Tell him everything you know. 
        You don't have to remember for him, you can give him partial information each time.
        
        Kwargs:

            numCones {int} -- How many iterations to be asleep before we send information to Control.

        """        
        # if already awake, don't check again, no matter what:
        if self._is_awake:
            return True

        # Possible Inputs:
        for key, value in kwargs.items(): 
            if key == 'numCones':
                self.lastCones = value

        self.lastIteration += 1
        self.lastTime = time.monotonic()

        # now check if awake:
        shouldWakeup = ( self.lastCones >= self.minCones ) and ( self.lastIteration >= self.minIterations ) and ( self.lastTime - self.InitTime >= self.minTime ) 
        if shouldWakeup:   
            self._is_awake = True # So next time we won't need to check this again.
            return True
        else:
            return False
        



if __name__ == "__main__":
    sm = SleeperManager( minTime = 0 , minCones = 4 , minIterations = 4 )
    res = sm.checkAwake(numCones = 2 )
    res = sm.checkAwake(numCones = 4 )
    res = sm.checkAwake(numCones = 4 )
    res = sm.checkAwake(numCones = 4 )
    print(f"Last results was =-={res}=-=")
    
