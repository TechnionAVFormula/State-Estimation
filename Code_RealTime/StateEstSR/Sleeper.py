
import time


class Sleeper():

    def __init__(self , **kwargs):
        self.InitTime = time.monotonic()
        #Default values:
        self._minTime = 0  #[sec]
        self._minCones = 0 #[number of detected coens]
        # Given Inputs:
        for key, value in kwargs.items(): 
            if key == 'minTime':
                self._minTime = value
            if key == 'minCones':
                self._minCones = value

    def checkAwake(self, numCones):
        pass




if __name__ == "__main__":
    a = IdleManager( minTime = 1.5 , minCones = 4 )
    print(a)
