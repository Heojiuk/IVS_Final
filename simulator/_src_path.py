import os, sys

def add():
    src = os.path.join(os.path.dirname(__file__), '..', 'src')
    src = os.path.normpath(src)
    if src not in sys.path:
        sys.path.insert(0, src)
