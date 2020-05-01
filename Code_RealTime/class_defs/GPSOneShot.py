class GPSOneShot:
    def __init__(self):
        self._x = 0
        self._y = 0
        self._time_milisec = 0
        self._is_new_data = False

    def set_new_data(self, x, y, time_milisec):
        self._x = x
        self._y = y
        self._time_milisec = time_milisec
        self._is_new_data = True

    def check_new_data(self):
        return self._is_new_data

    def get_data(self):
        self._is_new_data = False
        return self._x, self._y
