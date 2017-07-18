from xml.sax.saxutils import escape
import codecs

class MarkdownTablesOutput():
    def __init__(self, groups):
        result = ("# Parameter Reference\n"
                  "> **Note** **This list is auto-generated from the source code** and contains the most recent parameter documentation.\n"
                  "\n")
        for group in groups:
            result += '## %s\n\n' % group.GetName()
            
            #Check if scope (module where parameter is defined) is the same for all parameters in the group. 
            # If so then display just once about the table. 
            scope_set = set()
            for param in group.GetParams():
                scope_set.add(param.GetFieldValue("scope"))
            if len(scope_set)==1:
                result+='\nThe module where these parameters are defined is: *%s*.\n\n' %  list(scope_set)[0]
            
            
            result += '<table style="width: 100%; table-layout:fixed; font-size:1.5rem;">\n'
            result += ' <colgroup><col style="width: 23%"><col style="width: 46%"><col style="width: 11%"><col style="width: 11%"><col style="width: 9%"></colgroup>\n'
            result += ' <thead>\n'
            result += '   <tr><th>Name</th><th>Description</th><th>Min > Max (Incr.)</th><th>Default</th><th>Units</th></tr>\n'
            result += ' </thead>\n'
            result += '<tbody>\n'
            
            for param in group.GetParams():
                code = param.GetName()
                name = param.GetFieldValue("short_desc") or ''
                long_desc = param.GetFieldValue("long_desc") or ''
                min_val = param.GetFieldValue("min") or ''
                max_val = param.GetFieldValue("max") or ''
                increment = param.GetFieldValue("increment") or ''
                def_val = param.GetDefault() or ''
                unit = param.GetFieldValue("unit") or ''
                type = param.GetType()
                reboot_required = param.GetFieldValue("reboot_required") or ''
                #board = param.GetFieldValue("board") or '' ## Disabled as no board values are defined in any parameters!
                #decimal = param.GetFieldValue("decimal") or '' #Disabled as is intended for GCS not people
                #field_codes = param.GetFieldCodes() ## Disabled as not needed for display. 
                #boolean = param.GetFieldValue("boolean") # or '' # Disabled - does not appear useful.


                # Format values for display.
                # Display min/max/increment value based on what values are defined.
                max_min_combined = ''
                if min_val or max_val:
                    if not min_val:
                        min_val='?'
                    if not max_val:
                        max_val='?'
                    max_min_combined+='%s > %s ' % (min_val, max_val)
                if increment:
                    max_min_combined+='(%s)' % increment

                if long_desc is not '':
                    long_desc = '<p><strong>Comment:</strong> %s</p>' % long_desc

                if name == code:
                    name = ""
                code='<strong id="%s">%s</strong>' % (code, code)

                if reboot_required:
                    reboot_required='<p><b>Reboot required:</b> %s</p>\n' % reboot_required
                
                scope=''
                if not len(scope_set)==1 or len(scope_set)==0:
                    scope = param.GetFieldValue("scope") or ''
                    if scope:
                        scope='<p><b>Module:</b> %s</p>\n' % scope


                enum_codes=param.GetEnumCodes() or '' # Gets numerical values for parameter.
                enum_output=''
                # Format codes and their descriptions for display. 
                if enum_codes:
                    enum_output+='<strong>Values:</strong><ul>'
                    enum_codes=sorted(enum_codes,key=int)
                    for item in enum_codes:
                        enum_output+='\n<li><strong>%s:</strong> %s</li> \n' % (item, param.GetEnumValue(item))
                    enum_output+='</ul>\n'
                    

                bitmask_list=param.GetBitmaskList() #Gets bitmask values for parameter
                bitmask_output=''
                #Format bitmask values
                if bitmask_list:
                    bitmask_output+='<strong>Bitmask:</strong><ul>'
                    for bit in bitmask_list:
                        bit_text = param.GetBitmaskBit(bit)
                        bitmask_output+='  <li><strong>%s:</strong> %s</li> \n' % (bit, bit_text)
                    bitmask_output+='</ul>\n'

                    
                result += '<tr>\n <td style="vertical-align: top;">%s (%s)</td>\n <td style="vertical-align: top;"><p>%s</p>%s %s %s %s %s</td>\n <td style="vertical-align: top;">%s</td>\n <td style="vertical-align: top;">%s </td>\n <td style="vertical-align: top;">%s</td>\n</tr>\n' % (code,type,name, long_desc, enum_output, bitmask_output, reboot_required, scope, max_min_combined,def_val,unit)

            #Close the table.
            result += '</tbody></table>\n\n'

        self.output = result

    def Save(self, filename):
        with codecs.open(filename, 'w', 'utf-8') as f:
            f.write(self.output)
