##excel reading
import pandas as pd
import array


def excel_function():
    file_read = pd.read_excel("Sensors_Data.xlsx")
    file_read = pd.DataFrame(file_read).to_numpy()
    return file_read

