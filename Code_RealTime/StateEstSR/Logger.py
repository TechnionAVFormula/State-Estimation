import logging

def InitLogger():
    logger = logging.getLogger() # Use default logger (i.e. logging.info/logging.debug)
    logger.setLevel(logging.DEBUG)

    # create console handler and set level to INFO
    ch = logging.StreamHandler()
    ch.setLevel(logging.INFO)

    # Create log file handler and set level to DEBUG
    log_file_name = 'log.log'
    file_handler = logging.FileHandler(log_file_name , 'w')
    formatter = logging.Formatter("%(asctime)s %(levelname)s:%(name)s:%(message)s") # Format for the file log lines
    file_handler.setFormatter(formatter)
    file_handler.setLevel(logging.DEBUG)

    # Add the handlers to the logger
    logger.addHandler(ch)
    logger.addHandler(file_handler)

    return logger



if __name__ == "__main__":
    logger = InitLogger()

    logger.info("Hello")
    logger.debug("Help")