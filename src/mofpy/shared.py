from threading import Lock


class Shared:
    shared_values = dict()
    lock = Lock()

    def __init__(self):
        pass

    @staticmethod
    def add(key, val):
        Shared.lock.acquire(True)
        Shared.shared_values[key] = val
        Shared.lock.release()

    @staticmethod
    def remove(key):
        if key in Shared.shared_values:
            Shared.lock.acquire(True)
            del Shared.shared_values[key]
            Shared.lock.release()

    @staticmethod
    def get_required(key):
        return Shared.shared_values[key]

    @staticmethod
    def get(key, default_val=None):
        if key not in Shared.shared_values:
            return default_val
        return Shared.shared_values[key]

    @staticmethod
    def set(key, val):
        Shared.add(key, val)

    @staticmethod
    def has(key):
        return key in Shared.shared_values
