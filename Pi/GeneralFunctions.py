from Constants import Constants

'''
General functions that are used across the code
'''

#-------------------------------------------------------------------------------
'''
Converts seconds to milli-seconds
@param sec - number of seconds
'''
convertSecToMilliSec = lambda sec : sec * Constants.MILLI_SEC_IN_SEC

#-------------------------------------------------------------------------------
'''
Converts millis-seconds to seconds
@param ms - number of milli-seconds
'''
convertMilliSecToSec = lambda ms : ms / Constants.MILLI_SEC_IN_SEC

