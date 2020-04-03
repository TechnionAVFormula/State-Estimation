##excel reading
import pandas as pd
import array


def excel_function():
    file_read = pd.read_csv(
        r"C:\Users\orene\Documents\Formula\KalmanFilterPython2\State-Estimation\Kalmanfilter-python\Sensors_Data.csv"
    )
    file_read = pd.DataFrame(file_read).to_numpy()
    return file_read
