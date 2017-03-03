# interface for high level actions for ada_meal_scenario

class ActionException(Exception):
    def __init__(self, action, message):
        """ Wrap the name of the action into the message

        @param action: name of action (string)
        @param message: message (string)
        """

        msg = '[%s] %s' % (action.name, message)
        super(ActionException, self).__init__(msg)

class BypassableAction(object):
    
    def __init__(self, name, bypass=False):
        """ 
        @param bypass: Flag for bypassable actions
        """
        self.name = name
        self.bypass = bypass

    def execute(self, *args, **kw_args):
        
        if self.bypass:
            self._bypass(*args, **kw_args)
        else:
            self._run(*args, **kw_args)

