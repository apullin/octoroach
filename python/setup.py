from setuptools.command import easy_install

print "Checking python library dependencies:"
print "-------------------------------------\n"

def tryLibrary(importname, textname, url):
    try:
        exec('import ' + importname)
        print "[OK]   ",textname,"library present.\n"
    except:
        print "[!!]   Unable to open the",textname,"library."
        print "[!!]   Trying to use easy_install to install it now..."
        try:
            easy_install.main( ["-U",importname] )
            print "[OK]   ",textname,"library installed.\n"
        except:
            print "[!!]   Could not install ",textname,"library with easy_install."
            print "[!!]   Library available at:"
            print "        ",url


tryLibrary('xbee', 'Xbee', 'https://code.google.com/p/python-xbee/')
tryLibrary('serial', 'PySerial', 'http://pyserial.sourceforge.net/')
tryLibrary('numpy', 'NumPy', 'http://www.numpy.org/')
tryLibrary('pygame', 'PyGame', 'http://www.pygame.org/news.html')

