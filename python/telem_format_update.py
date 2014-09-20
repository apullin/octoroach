import sys

if len(sys.argv) != 2:
    print "Wrong number of arguments."
    print "  usage: telem_format_update.py <xx_telem.h>"
    sys.exit(1)

savetypes = {"int":"%d", "int16_t":"%d",
         "unsigned int":"%d", "uint16_t":"%d",
         "long":"%d", "int32_t":"%d",
         "unsigned long":"%d", "int32_t":"%d",
         "float":"%f",
         "char":"%s"}
         
unpacktypes = {"int":"h", "int16_t":"h",
         "unsigned int":"H", "uint16_t":"H",
         "long":"l", "int32_t":"l",
         "unsigned long":"L", "uint32_t":"L",
         "float":"f",
         "char":"s"}
         
       

headername = sys.argv[1]
try:
    file = open(headername)
except IOError:
    print "Unable to open ",headername
    sys.exit(1)

line = file.readline().strip()
while line != "//START_PARSE":
    line = file.readline().strip()
    
decls = []    

line = file.readline().strip()
while line != "//END_PARSE":
    decls.append(line)
    line = file.readline().strip()

#print decls

savestring = []
unpackstring = []
for item in decls:
    temp = item.strip(';')
    temp = item.split(' ')
    numThisType = len(temp) - 1
    savestring.append(savetypes[temp[0]])
    unpackstring.append(unpacktypes[temp[0]])
    
print "telem save format string = "
print "    ",
print "".join(savestring)
print ""
print "callback telem unpack:"
print "    ",
print "".join(unpackstring)