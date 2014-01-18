from __future__ import with_statement
import re
import sys
import webbrowser

class Symbol:
    def __init__(self, section, address, size, file, name):
        self.address = address
        self.size = size
        self.file = file
        self.name = name
        self.section = section
    def __str__(self):
        return self.name
        
def split_section(str):
    chunks = str.split('.')
    if len(chunks) > 2:
        return (chunks[1], chunks[2])   
    else:
        return (chunks[1], "")

symbols = []

f = open(sys.argv[1])
linecount = 0
while True:
    line = f.readline()
    linecount += 1
    if line == None:
        break
    if line[0:28] == "Linker script and memory map":
        break

sym = ["", "", 0, 0, ""]
sym_sect = ""
sym_name = ""
sym_file = ""
sym_size = 0
sym_address = 0
sym_prevaddress = 0
sym_startaddress = 0
tokens = []
n = 0
state = 0
line = ""
while True:
    if n == len(tokens):
        line = f.readline()
        linecount += 1
        if not line:
            break;
        tokens = line.split()
        n = 0
        continue
        
    #print "%d: in state %d with token %s" % (linecount, state, tokens[n])
    
    if state == 0:
        if line != None and len(line) > 0:
            if line[0:2] == " .":
                state = 1
            else:
                n = len(tokens)
        else:
            n = len(tokens)
    elif state == 1: #expect section and symbol name
        (sym_sect, sym_name) = split_section(tokens[n])
        n += 1
        state += 1
    elif state == 2: #expect address
        try:
            sym_address = int(tokens[n], 0)
        except ValueError:
            state = 0
            continue
        sym_prevaddress = sym_address
        sym_startaddress = sym_address
        n += 1
        state += 1
    elif state == 3: #expect size
        sym_size = eval(tokens[n])
        n += 1
        state += 1
    elif state == 4: #expect file name
        sym_file = " ".join(tokens[n:])
        n = len(tokens)
        state += 1
        #symbols.append(Symbol(sym_sect, sym_startaddress, sym_size, sym_file, sym_name))
    elif state == 5: #expect new line or next function in current section
        if n == 0 and line != None and len(line) > 0 and (line[0:2] == " ." or line[0:2] == " *"):
            symbols.append(Symbol(sym_sect, sym_address, sym_size - sym_address + sym_startaddress, sym_file, sym_name))
            sym_prevaddress = sym_address
            state = 0
        else:
            state = 6
    elif state == 6: #expect next address
        try:
            sym_address = int(tokens[n], 0)
        except ValueError:
            state = 0
            continue
        if sym_prevaddress != sym_address:
            symbols.append(Symbol(sym_sect, sym_address, sym_address - sym_prevaddress, sym_file, sym_name))
            sym_prevaddress = sym_address
        n += 1
        state += 1
    elif state == 7: #expect next symbol name
        sym_name = " ".join(tokens[n:])
        n = len(tokens)
        state = 5

f.close()
        
# Sort
if ('size' in sys.argv):
    symbols.sort(lambda a,b: a.size - b.size)
else:
    symbols.sort(lambda a,b: a.address - b.address)

        
#===============================
# Gererate the HTML File
#

sections = ["text", "bss", "data"]
colors = ['9C9F84', 'A97D5D', 'F7DCB4']

max_height = 5.0
min_height = 0.3
bytes_to_em = 100.0

total_size = 0.0
for s in symbols:
    total_size += s.size

out = open(sys.argv[2], 'w')
	
out.write("<html><head>")
out.write("  <style>a { color: black; text-decoration: none; font-family:monospace }</style>")
out.write("<body>")
out.write("<table cellspacing='1px'>")
for sym in symbols:
    height = sym.size / bytes_to_em
    if height > max_height:
        height = max_height
    elif height < min_height:
        height = min_height
    font_size = 1.0 if height > 1.0 else height

    if sym.section not in sections:
        continue
    
    out.write("<tr style='white-space:nowrap;background-color:#%s;height:%gem;line-height:%gem;font-size:%gem'><td style='overflow:hidden'>" % \
        (colors[sections.index(sym.section)], height, height, font_size))
    out.write("Section: %s | Name: %s | Address: 0x%x | Size: %d | File: %s" % (sym.section, sym.name, sym.address, sym.size, sym.file))
    out.write("</td></tr>")
out.write("</table>")
out.write("</body></html>")

if 'open' in sys.argv:
	url = sys.argv[0]
	url = url.split('\\')
	url = '/'.join(url[0:-1])
	url = "file:///" + url + '/' + sys.argv[2]
	print url
	webbrowser.open(url, new = 2)