#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2013-2014 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

#
# PX4 paramers processor (main executable file)
#
# This tool scans the PX4 source code for declarations of tunable parameters
# and outputs the list in various formats.
#
# Currently supported formats are:
#   * XML for the parametric UI generator
#   * Human-readable description in DokuWiki page format
#
# This tool also allows to automatically upload the human-readable version
# to the DokuWiki installation via XML-RPC.
#

from __future__ import print_function
import sys
import os
import argparse
from px4params import srcscanner, srcparser, xmlout, dokuwikiout, dokuwikirpc, markdownout

import re
import json
import codecs

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="Process parameter documentation.")
    parser.add_argument("-s", "--src-path",
                        default=["../src"],
                        metavar="PATH",
                        nargs='*',
                        help="one or more paths to source files to scan for parameters")
    parser.add_argument("-x", "--xml",
                        nargs='?',
                        const="parameters.xml",
                        metavar="FILENAME",
                        help="Create XML file"
                             " (default FILENAME: parameters.xml)")
    parser.add_argument("-i", "--inject-xml",
                        nargs='?',
                        const="../Tools/parameters_injected.xml",
                        metavar="FILENAME",
                        help="Inject additional param XML file"
                             " (default FILENAME: ../Tools/parameters_injected.xml)")
    parser.add_argument("-b", "--board",
                         nargs='?',
                         const="",
                         metavar="BOARD",
                         help="Board to create xml parameter xml for")
    parser.add_argument("-m", "--markdown",
                        nargs='?',
                        const="parameters.md",
                        metavar="FILENAME",
                        help="Create Markdown file"
                             " (default FILENAME: parameters.md)")
    parser.add_argument("-w", "--wiki",
                        nargs='?',
                        const="parameters.wiki",
                        metavar="FILENAME",
                        help="Create DokuWiki file"
                             " (default FILENAME: parameters.wiki)")
    parser.add_argument("-u", "--wiki-update",
                        nargs='?',
                        const="firmware:parameters",
                        metavar="PAGENAME",
                        help="Update DokuWiki page"
                             " (default PAGENAME: firmware:parameters)")
    parser.add_argument("--wiki-url",
                        default="https://pixhawk.org",
                        metavar="URL",
                        help="DokuWiki URL"
                             " (default: https://pixhawk.org)")
    parser.add_argument("--wiki-user",
                        default=os.environ.get('XMLRPCUSER', None),
                        metavar="USERNAME",
                        help="DokuWiki XML-RPC user name"
                             " (default: $XMLRPCUSER environment variable)")
    parser.add_argument("--wiki-pass",
                        default=os.environ.get('XMLRPCPASS', None),
                        metavar="PASSWORD",
                        help="DokuWiki XML-RPC user password"
                             " (default: $XMLRPCUSER environment variable)")
    parser.add_argument("--wiki-summary",
                        metavar="SUMMARY",
                        default="Automagically updated parameter documentation from code.",
                        help="DokuWiki page edit summary")
    parser.add_argument('-v', '--verbose', action='store_true', help="verbose output")
    parser.add_argument("-o", "--overrides",
                        default="{}",
                        metavar="OVERRIDES",
                        help="a dict of overrides in the form of a json string")


    args = parser.parse_args()

    # Check for valid command
    if not (args.xml or args.wiki or args.wiki_update or args.markdown):
        print("Error: You need to specify at least one output method!\n")
        parser.print_usage()
        sys.exit(1)

    # Initialize source scanner and parser
    scanner = srcscanner.SourceScanner()
    parser = srcparser.SourceParser()

    # Scan directories, and parse the files
    if (args.verbose): print("Scanning source path " + str(args.src_path))
    
    if not scanner.ScanDir(args.src_path, parser):
        sys.exit(1)

    if not parser.Validate():
        sys.exit(1)
    param_groups = parser.GetParamGroups()

    if len(param_groups) == 0:
        print("Warning: no parameters found")

    override_dict = json.loads(args.overrides)
    if len(override_dict.keys()) > 0:
        for group in param_groups:
            for param in group.GetParams():
                name = param.GetName()
                if name in override_dict.keys():
                    val = str(override_dict[param.GetName()])
                    param.default = val
                    print("OVERRIDING {:s} to {:s}!!!!!".format(name, val))

    # Output to XML file
    if args.xml:
        if args.verbose: print("Creating XML file " + args.xml)
        cur_dir = os.path.dirname(os.path.realpath(__file__))
        out = xmlout.XMLOutput(param_groups, args.board,
                               os.path.join(cur_dir, args.inject_xml))
        out.Save(args.xml)

    # Output to DokuWiki tables
    if args.wiki or args.wiki_update:
        out = dokuwikiout.DokuWikiTablesOutput(param_groups)
        if args.wiki:
            print("Creating wiki file " + args.wiki)
            out.Save(args.wiki)
        if args.wiki_update:
            if args.wiki_user and args.wiki_pass:
                print("Updating wiki page " + args.wiki_update)
                xmlrpc = dokuwikirpc.get_xmlrpc(args.wiki_url, args.wiki_user, args.wiki_pass)
                xmlrpc.wiki.putPage(args.wiki_update, out.output, {'sum': args.wiki_summary})
            else:
                print("Error: You need to specify DokuWiki XML-RPC username and password!")

    # Output to Markdown/HTML tables
    if args.markdown:
        out = markdownout.MarkdownTablesOutput(param_groups)
        if args.markdown:
            print("Creating markdown file " + args.markdown)
            out.Save(args.markdown)

    #print("All done!")


if __name__ == "__main__":
    main()
