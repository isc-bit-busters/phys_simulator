import os



def getPath(path)->str:
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), path)