# pylint

Contains the `pylintr` which we use to style check in CI, as well as any custom
checkers. 

## Custom checkers

- `todo.py`: Checks that every `TODO` declared in comments have an owner
  assigned

## Available Messages

The following are all default, available messages that come with pylint. We are
using most, but not all of them.

- C0102: Black listed name "%s"
- C0103: Invalid %s name "%s"
- C0111: Missing %s docstring
- C0112: Empty %s docstring
- C0121: Missing required attribute "%s"
- C0202: Class method %s should have cls as first argument
- C0203: Metaclass method %s should have mcs as first argument
- C0204: Metaclass class method %s should have %s as first argument
- C0301: Line too long (%s/%s)
- C0302: Too many lines in module (%s)
- C0303: Trailing whitespace
- C0304: Final newline missing
- C0322: Old: Operator not preceded by a space
- C0323: Old: Operator not followed by a space
- C0324: Old: Comma not followed by a space
- C0325: Unnecessary parens after %r keyword
- C0326: %s space %s %s %s\n%s
- C1001: Old-style class defined.
- E0001: (syntax error raised for a module; message varies)
- E0011: Unrecognized file option %r
- E0012: Bad option value %r
- E0100: __init__ method is a generator
- E0101: Explicit return in __init__
- E0102: %s already defined line %s
- E0103: %r not properly in loop
- E0104: Return outside function
- E0105: Yield outside function
- E0106: Return with argument inside generator
- E0107: Use of the non-existent %s operator
- E0108: Duplicate argument name %s in function definition
- E0202: An attribute affected in %s line %s hide this method
- E0203: Access to member %r before its definition line %s
- E0211: Method has no argument
- E0213: Method should have "self" as first argument
- E0221: Interface resolved to %s is not a class
- E0222: Missing method %r from %s interface
- E0235: __exit__ must accept 3 arguments: type, value, traceback
- E0501: Old: Non ascii characters found but no encoding specified (PEP 263)
- E0502: Old: Wrong encoding specified (%s)
- E0503: Old: Unknown encoding specified (%s)
- E0601: Using variable %r before assignment
- E0602: Undefined variable %r
- E0603: Undefined variable name %r in __all__
- E0604: Invalid object %r in __all__, must contain only strings
- E0611: No name %r in module %r
- E0701: Bad except clauses order (%s)
- E0702: Raising %s while only classes, instances or string are allowed
- E0710: Raising a new style class which doesn't inherit from BaseException
- E0711: NotImplemented raised - should raise NotImplementedError
- E0712: Catching an exception which doesn\'t inherit from BaseException: %s
