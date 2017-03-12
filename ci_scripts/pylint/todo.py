#pylint: skip-file
# Copyright (c) 2006, 2009-2013 LOGILAB S.A. (Paris, FRANCE) <contact@logilab.fr>
# Copyright (c) 2013-2014 Google, Inc.
# Copyright (c) 2014 Alexandru Coman <fcoman@bitdefender.com>
# Copyright (c) 2014-2016 Claudiu Popa <pcmanticore@gmail.com>

# Licensed under the GPL: https://www.gnu.org/licenses/old-licenses/gpl-2.0.html
# For details: https://github.com/PyCQA/pylint/blob/master/COPYING

#Modified to handle custom todo messages, taken from pylint source

"""Check every 'TODO' declared in comments have an owner assigned"""

# pylint: disable=W0511

import re

import six

from pylint.interfaces import IRawChecker
from pylint.checkers import BaseChecker


MSGS = {
    'W9000': ('todo has no owner: \"%s\"',
              'unowned-todo',
              'Used to indicate when a todo with no user has been detected'),
    'W9001': ('Cannot decode using encoding "%s", unexpected byte at position %d',
              'todo-invalid-encoded-data',
              'Used when a source line cannot be decoded using the specified '
              'source file encoding.',
              {'maxversion': (3, 0)}),
}


class CustomTODOChecker(BaseChecker):

    """checks for:
    * TODO(<username>) in the source
    * encoding issues.
    """
    __implements__ = IRawChecker

    # configuration section name
    name = 'unowned-todo'
    msgs = MSGS

    ignore = re.compile(r"TODO\(.+\)")

    def _check_note(self, notes, lineno, line):

        match = notes.search(line)
        if not match:
            return
        if not self.ignore.search(line):
            self.add_message('unowned-todo', args=line[match.start(1):-1], line=lineno)

    def _check_encoding(self, lineno, line, file_encoding):
        try:
            return six.text_type(line, file_encoding)
        except UnicodeDecodeError as ex:
            self.add_message('invalid-encoded-data', line=lineno,
                             args=(file_encoding, ex.args[2]))

    def process_module(self, module):
        """inspect the source file to find encoding problem or fixmes like
        notes
        """
        notes = re.compile(r'.*?#\s*(%s)(:*\s*.*)' % "TODO")

        if module.file_encoding:
            encoding = module.file_encoding
        else:
            encoding = 'ascii'

        with module.stream() as stream:
            for lineno, line in enumerate(stream):
                line = self._check_encoding(lineno + 1, line, encoding)
                if line is not None and notes:
                    self._check_note(notes, lineno + 1, line)


def register(linter):
    """required method to auto register this checker"""
    linter.register_checker(CustomTODOChecker(linter))
