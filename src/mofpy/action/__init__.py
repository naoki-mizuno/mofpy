# Import only the base class. Automatically import the rest
from .action import Action

import pkgutil

__path__ = pkgutil.extend_path(__path__, __name__)
for importer, modname, ispkg in pkgutil.walk_packages(path=__path__,
                                                      prefix=__name__ + '.'):
    try:
        __import__(modname)
    except ImportError as e:
        # Silently fail here. Message will be printed by PresetHandler
        Action.disabled[modname] = e.message

del importer, modname, ispkg
del pkgutil
