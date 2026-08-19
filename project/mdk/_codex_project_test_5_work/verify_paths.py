# -*- coding: utf-8 -*-
"""Compatibility entry point for the independent strict regression runner.

The previous verifier imported the solver it claimed to be independent from and
did not validate observation direction, bomb rules, completed-box removal, or
the final car position.  Keep the familiar filename, but route every invocation
through the rule-complete replay engine.
"""

from strict_regression import main


if __name__ == '__main__':
    raise SystemExit(main())
