import symengine as se

def parse_value(value_str):
    """
    Helper function to parse a value from the netlist.
    Handles scientific notation and standard SPICE unit suffixes (K, M, U, N, P, etc.).
    Converts values to symengine.Rational for precision and performance.
    If it's not a valid number (e.g., "SYMBOLIC"), returns a symengine.Symbol.
    """
    # Ensure the input is a string, then convert to uppercase for case-insensitive matching.
    value_str = str(value_str).upper()
    
    # SPICE unit suffixes, following standard conventions (M=milli, MEG=Mega).
    units = {
        'T': 1e12,
        'G': 1e9,
        'MEG': 1e6,
        'K': 1e3,
        'M': 1e-3,
        'U': 1e-6,
        'N': 1e-9,
        'P': 1e-12,
        'F': 1e-15,
    }

    # Sort keys by length, longest first, to handle 'MEG' before 'M' or 'G'.
    # This ensures the most specific suffix is matched first.
    sorted_suffixes = sorted(units.keys(), key=len, reverse=True)

    # Check for unit suffixes.
    for suffix in sorted_suffixes:
        if value_str.endswith(suffix):
            numeric_part = value_str[:-len(suffix)]
            try:
                # Attempt to convert the numeric part of the string to a float.
                value = float(numeric_part) * units[suffix]
                # Convert to symengine.Rational for maximum precision in symbolic calculations.
                return se.Rational(str(value))
            except (ValueError, TypeError):
                # --- FIX ---
                # If conversion fails, it means this suffix was not the correct one
                # (e.g., matching 'G' in 'MEG'). Continue to the next suffix.
                # Previously, this was a 'break', which prematurely exited the loop.
                continue

    # If no suffix was matched or parsed successfully, try to convert the whole string.
    try:
        return se.Rational(value_str)
    except (ValueError, TypeError, SyntaxError):
        # If all parsing attempts fail, treat it as a symbolic variable.
        return se.Symbol(value_str)

def format_as_poly(expr, s):
    """Formats a rational expression as a ratio of polynomials in s."""
    try:
        # Separate numerator and denominator
        num, den = se.fraction(se.expand(expr))
        
        # Collect terms for numerator and denominator
        num_poly = se.Poly(num, s)
        den_poly = se.Poly(den, s)
        
        # Format them nicely
        num_str = str(num_poly.as_expr()).replace('Poly(', '').replace(', s, domain=ZZ)', '')
        den_str = str(den_poly.as_expr()).replace('Poly(', '').replace(', s, domain=ZZ)', '')

        # Avoid showing '/ 1' for integer expressions
        if den_str == "1":
            return f"({num_str})"
        else:
            return f"({num_str}) / ({den_str})"
    except Exception:
        # Fallback for expressions that are not rational polynomials in s
        return str(expr)
