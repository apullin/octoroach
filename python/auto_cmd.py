#!/usr/bin/env python
"""
authors: apullin

Contents of this file are copyright Andrew Pullin, 2013

"""

from lib import command

#fhHeader = open('../firmware/source/cmd.h')

# pretty printing justification math
namewidth = max( [len(item.cmdName) for item in command.cmdList] ) 
funcwidth = max( [len(item.cmdFunction) for item in command.cmdList] )

defStr = "#define "
defwidth = len(defStr)

print "For cmd_user.h:"
for item in command.cmdList:
    hexStr = str("0x%02X" % item.cmdNum)
    outStr = defStr + "%s " % item.cmdName
    print "%s %s" % (outStr.ljust(namewidth + defwidth + 1) , hexStr)


            
print
print "For cmd_user.c:"
for item in command.cmdList:
    print "cmdSetUserCommand(NAMESPACE_USER, %s, %s, MAX_PAYLOAD_LENGTH)" % (item.cmdName.rjust(namewidth) , item.cmdFunction.rjust(funcwidth))
    
#fhHeader.close()