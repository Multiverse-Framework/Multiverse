import os
if os.name == "posix":
    from .socket_service import SocketService
