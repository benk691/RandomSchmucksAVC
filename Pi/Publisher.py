
class Publisher:
  '''
  Publishes data out to its subscribers
  '''
  #-------------------------------------------------------------------------------
  def __init__(self):
    '''
    Initializes the publisher
    '''
    self.subscribers = dict()

  #-------------------------------------------------------------------------------
  def register(self, subscriber, callback):
    '''
    Registers a subsriber to the publisher
    @param subscriber - the class that is subscribing to this publisher
    @param callback - function to call upon state update
    '''
    self.subscribers[subscriber] = callback

  #-------------------------------------------------------------------------------
  def unregister(self, subscriber):
    '''
    Unregisters a subscriber from the publisher
    @param subscriber - class to unregister
    '''
    del self.subscribers[subscriber]

  #-------------------------------------------------------------------------------
  def publish(self, *args):
    '''
    Publishes arguments to all its subscribers
    @param args - list of arguments to upate all the subscribers with
    '''
    for subscriber, callback in self.subscribers.items():
      callback(*args)

  

