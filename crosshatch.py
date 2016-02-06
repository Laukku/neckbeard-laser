  # Experimental crosshatch rastering method. The image is built up of diagonal lines going both ways. I've seen a nice example of this live and I wanted to accomplish the same, but this code is not finished yet... I think it makes the paths correctly, but I can't remember anymore. I haven't touched the code in a while.
  def Crosshatch(self, id, rasterspeed=None, resolution=None, max_power=None):
        cross_gcode = ''
        if rasterspeed==None:
            rasterspeed = self.options.rasterspeed
        if resolution==None or resolution <= 0:
            resolution = self.options.resolution
        if max_power==None or max_power <= 0:
            max_power = self.options.laser_max_value    
        # TODO: What options should be accepted from the layer name parsing?
        DPI = resolution * 25.4
        
        cross_gcode += ';DPI: %s; That means %s lines/\"pixels\" per mm \n' %(DPI, resolution)
        current_file = self.args[-1]
        exported_png = self.getTmpPath() + "laser_temp.png"
        cross_gcode += '; ' + exported_png + "\n"
        
        # Get coordinates of object: lower left? by querying inkscape
        command1 = "inkscape \"%s\" --query-id \"%s\" --query-x " % (current_file, id)
        command2 = "inkscape \"%s\" --query-id \"%s\" --query-y " % (current_file, id)
        command3 = "inkscape \"%s\" --query-id \"%s\" --query-width " % (current_file, id)
        command4 = "inkscape \"%s\" --query-id \"%s\" --query-height " % (current_file, id)
        command = command1 + '; echo ;' + command2 + '; echo ;' + command3 + '; echo ;' + command4
        p = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        return_code = p.wait()
        # Parse coordinates into variables for later use
        coords = p.stdout.read().splitlines()
        inkex.errormsg(str(coords) + '\n')
        

        
        # TODO: Convert to topleft coordinates - they already are?
        # Export image data from svg based on coordinates
        
        command = "inkscape \"%s\" -i \"%s\" -j -b \"%s\" --export-png=\"%s\" -d %s" % (
            current_file, id, self.options.bg_color, exported_png, DPI)
        #command = "inkscape \"%s\" -i \"%s\" -j -b \"%s\" -C --export-png=\"%s\" -d %s" % (
        #    current_file, id, self.options.bg_color, exported_png, DPI)
        # command="inkscape -C -e \"%s\" -b\"%s\" %s -d %s" % (exported_png, bg_color, current_file, DPI)

        p = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        return_code = p.wait()
        f = p.stdout
        err = p.stderr

        ######## Open the image that was exported with Inkscape
        reader = png.Reader(exported_png)
        w, h, pixels, metadata = reader.read_flat()

        ######## Make an array containing the image in greyscale, reducing color depth as requested
        gray_array = [[255 for i in range(w)] for j in range(h)]
        for y in range(h):
            for x in range(w):
                pos = (x + y * w) * 4 if metadata['alpha'] else (x + y * w) * 3
                # A natural looking grayscale conversion, where the R, G and B channels all affect the grayscale value a different amount. There is no alpha in the source file.
                avg = int(pixels[pos] * 0.21 + pixels[pos + 1] * 0.72 + pixels[pos + 2] * 0.07)
                # Reduce color depth, maintain scale of 0-255
                reduced = int((int((avg/(float(256)/self.options.greyscale_depth)))) * (float(255) / (self.options.greyscale_depth -1)))
                gray_array[y][x] = reduced


        ####### Make GCode from image data
        #if self.options.flip_y == False:
        #    gray_array.reverse()
        # Probably not going to work if y is flipped
        Laser_ON = False

        F_G01 = rasterspeed
        F_G00 = self.options.Mfeed
        scale = resolution  

        # HOMING
        if self.options.homing == 1:
            cross_gcode += 'G28; home all axes\n'
        elif self.options.homing == 2:
            cross_gcode += '$H; home all axes\n'
        else:
            pass
        cross_gcode += 'G21; Set units to millimeters\n'
        cross_gcode += 'G90; Use absolute coordinates\n'
        cross_gcode += 'G92; Coordinate Offset\n'

        ########## Cross-hatching with look ahead, by Lauri Niskanen
        # Compacts Gcode by combining consecutive pixels with the same value into a single command.
        # Converts grayscale range into laser intensity values suitable for the user's laser machine.
        # Cuts in a diagonal pattern, first up left <-> down right, then up right <-> down left
        
        # Factor to reduce laser intensity for cross-hatching, since every pixel is visited twice and stuff might get pretty burnt up!
        factor = 0.5
        def intensity(pix): # Calculate laser intensity value
            return (max_power - (
            ((max_power - self.options.laser_min_value) * pix) / 255)) * factor
        
        
        # Begin cross-hatching:
        xmin = 0
        ymin = 0
        xmax = len(gray_array)-1
        ymax = len(gray_array[y])-1
        x = 0
        y = ymax
        movelist = [[True,x,y,255]] # G0, x, y, white
        # First pass: up left - down right motion
        # Moves are first saved in a temporary list on a per pixel basis, and later compacted into linear  Gcode moves.
        
        f = open(self.getTmpPath() + "laser_temp.txt",'w') # open debug file
        #try:
        while not (x == xmax and y == ymin) and x>=xmin and x<=xmax and y>=ymin and y<=ymax:
            if y==ymax:
                if x==xmax:
                    y -= 1
                else:
                    x += 1
                movelist.append([True,x,y,gray_array[y][x]])
                x -= 1
                y -= 1
                movelist.append([False,x,y,gray_array[y][x]])
                going="up"
            elif x==xmin:
                if y==ymin:
                    x += 1
                else:
                    y -= 1
                movelist.append([True,x,y,gray_array[y][x]])
                x += 1
                y += 1
                movelist.append([False,x,y,gray_array[y][x]])
                going="down"
            elif x==xmax and not y==ymin:
                y -= 1
                movelist.append([True,x,y,gray_array[y][x]])
                if not y==ymin:
                    x -= 1
                    y -= 1
                    movelist.append([False,x,y,gray_array[y][x]])
                    going="up"
            elif y==ymin and not x==xmax:
                x += 1
                movelist.append([True,x,y,gray_array[y][x]])
                if not x==xmax:
                    x += 1
                    y += 1
                    movelist.append([False,x,y,gray_array[y][x]])
                    going="down"
            elif going=='down' and x > xmin and x < xmax and y > ymin and y < ymax:
                x += 1
                y += 1
                movelist.append([False,x,y,gray_array[y][x]])
            elif going=='up' and x > xmin and x < xmax and y > ymin and y < ymax:
                x -= 1
                y -= 1
                movelist.append([False,x,y,gray_array[y][x]])
            #except IndexError,e:
                
        # Unpack movelist to a Gcode file
        p=0
        while p < len(movelist):

            if movelist[p][3] <= self.options.white_cutoff:
                # Grab pixel value
                compare_to = movelist[p][3]
                start = p
                # True means G0 or move along the sides in movelist[p][0]
                
                # G0 to this point but only if needed:
                # If the end point of the last move is not the same as the beginning of this move, i.e. there was some whitespace in between
                # or if this is a move along the side
                if not start == end or movelist[p][0]:
                    cross_gcode += 'G0 X' + "{0:.3f}".format((float(movelist[p][1]) / scale)+float(coords[0])) + ' Y' + "{0:.3f}".format((float(movelist[p][2]) / scale)+float(coords[1])) + ' F' + str(F_G00) + '\n'
                
                # loop till end of similar pixels or EOL
                while p < len(movelist) and compare_to == movelist[p]:
                    p += 1  # Every G1 move is at least 1 pixel long
                end = p # This move ends here

                # Calculate laser intensity
                l = intensity(compare_to)
                # G1 to that point
                cross_gcode += 'G1 X' + "{0:.3f}".format((float(movelist[p][1]) / scale) + float(coords[0])) + ' Y' + "{0:.3f}".format((float(movelist[p][2]) / scale)+float(coords[1])) + ' F' + str(F_G01) + ' S' + str(l) + '\n'
            else:
                p += 1
        
        
        # Second pass: down left - up right motion
        x = 0
        y = 0
        movelist2 = [[True,x,y,255]] # G0, x, y, white
        # TODO: Fix the directions here - just change xmax,min etc?
        while not (x == xmax and y == ymax) and x>=xmin and x<=xmax and y>=ymin and y<=ymax:
            if y==ymin:
                if x==xmax:
                    y += 1
                else:
                    x += 1
                movelist2.append([True,x,y,gray_array[y][x]])
                x -= 1
                y += 1
                movelist2.append([False,x,y,gray_array[y][x]])
                going="down"
            elif x==xmin:
                if y==ymax:
                    x += 1
                else:
                    y += 1
                movelist2.append([True,x,y,gray_array[y][x]])
                x += 1
                y -= 1
                movelist2.append([False,x,y,gray_array[y][x]])
                going="up"
            elif x==xmax and not y==ymax:
                y += 1
                movelist2.append([True,x,y,gray_array[y][x]])
                if not y==ymax:
                    x -= 1
                    y += 1
                    movelist2.append([False,x,y,gray_array[y][x]])
                    going="down"
            elif y==ymax and not x==xmax:
                x += 1
                movelist2.append([True,x,y,gray_array[y][x]])
                if not x==xmax:
                    x += 1
                    y -= 1
                    movelist2.append([False,x,y,gray_array[y][x]])
                    going="up"
            elif going=='up' and x > xmin and x < xmax and y > ymin and y < ymax:
                x += 1
                y -= 1
                movelist2.append([False,x,y,gray_array[y][x]])
            elif going=='down' and x > xmin and x < xmax and y > ymin and y < ymax:
                x -= 1
                y += 1
                movelist2.append([False,x,y,gray_array[y][x]])
                movelist
        # Unpack movelist to a Gcode file
        p=0
        while p < len(movelist2):
    
            if movelist2[p][3] <= self.options.white_cutoff:
                # Grab pixel value
                compare_to = movelist2[p][3]
                start = p
                # True means G0 = move along the sides in movelist[p][0]
                
                # G0 to this point but only if needed:
                # If the end point of the last move is not the same as the beginning of this move, i.e. there was some whitespace in between
                # or if this is a move along the side
                if not start == end or movelist2[p][0]:
                    cross_gcode += 'G0 X' + "{0:.3f}".format((float(movelist2[p][1]) / scale)+float(coords[0])) + ' Y' + "{0:.3f}".format((float(movelist2[p][2]) / scale)+float(coords[1])) + ' F' + str(F_G00) + '\n'
                
                # loop till end of similar pixels or EOL
                while p < len(movelist2) and compare_to == movelist2[p]:
                    p += 1  # Every G1 move is at least 1 pixel long
                end = p # This move ends here

                # Calculate laser intensity
                l = intensity(compare_to)
                # G1 to that point
                cross_gcode += 'G1 X' + "{0:.3f}".format((float(movelist2[p][1]) / scale)+float(coords[0])) + ' Y' + "{0:.3f}".format((float(movelist2[p][2]) / scale)+float(coords[1])) + ' F' + str(F_G01) + ' S' + str(l) + '\n'
            else:
                p += 1
        
        
        
        # Debug code:
            
        #for line in movelist:
            #f.write(going + ' ' + str(line) + '\n')    
            
        #f.close() # close debug file
        
        cross_gcode += "M5\n"
        return cross_gcode
        
