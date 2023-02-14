import datetime


# busy => I will do the thing once im done
# or   => I will ignore the thing you send while I was busy

# sync_factor should not be less than 0.05
# used to make sure non-expired (non-request) events to register and run
# sync_factor is used to component for the time delay between event being created and getting here
# Over compensation of sync_factor might result expired events (with the sync_factor - created when the
# ** action is almost done) to register and run.
def atomic(sync_factor=0.05, device=None):
    sync_factor = 0.05 if sync_factor < 0.05 else sync_factor

    def decorator(fun):
        def wrapper(*args, **kwargs):
            last_work_done_time = datetime.datetime.now()
            if last_work_done_time - datetime.timedelta(seconds=sync_factor) < args[1].get('time'):
                fun(*args, **kwargs)
            else:
                print()
                # print(f"Error: **atomic** Event Dropped by '{device if device else ''}': {args[1]}")
                
                print()

        return wrapper

    return decorator
