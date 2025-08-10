import symengine as se

def parse_value(value_str):
    """
    Helper function to parse a value from the netlist using explicit text processing.
    This function first attempts to parse a number with a SPICE unit suffix.
    If that fails, it tries to parse a plain number.
    If all numeric parsing fails, it returns a symbolic variable.
    """
    # 1. Sanitize input for consistent processing
    value_str_upper = str(value_str).strip().upper()
    if not value_str_upper:
        return se.Symbol("EMPTY_VALUE") # Handle empty strings

    # 2. Define SPICE unit suffixes and their multipliers
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

    # Sort keys by length (longest first) to correctly handle 'MEG' before 'M'
    sorted_suffixes = sorted(units.keys(), key=len, reverse=True)

    # 3. --- Text Processing Logic ---
    # Attempt to parse the string as a number with a unit suffix.
    for suffix in sorted_suffixes:
        if value_str_upper.endswith(suffix):
            numeric_part = value_str_upper[:-len(suffix)]
            try:
                # Convert the numeric part of the string to a float
                value = float(numeric_part)
                # Multiply by the unit's value
                final_value = value * units[suffix]
                # Return the result as a symengine numeric type
                return se.sympify(final_value)
            except (ValueError, TypeError):
                # If float() fails (e.g., "1K2" -> numeric_part="1K"), this suffix
                # was incorrect. Continue to the next possible suffix.
                continue

    # 4. --- Plain Number Fallback ---
    # If no unit suffix was found or parsed successfully, try parsing the whole string as a number.
    # This handles values without units, like "100" or "1.23e-4".
    try:
        plain_value = float(value_str_upper)
        return se.sympify(plain_value)
    except (ValueError, TypeError):
        # The string is not a plain number.
        pass

    # 5. --- Symbolic Fallback ---
    # If all numeric parsing attempts fail, treat the entire string as a symbol.
    return se.Symbol(value_str_upper)

def format_as_poly(expr, s):
    """
    Formats a rational expression as a ratio of polynomials in s.
    Uses robust API calls instead of string manipulation.
    """
    try:
        # Cancel terms in the fraction first for simplification.
        simplified_expr = se.cancel(expr)
        num, den = se.fraction(simplified_expr)
        
        # Use .as_expr() to reliably get the polynomial expression.
        num_expr = se.Poly(num, s).as_expr()
        den_expr = se.Poly(den, s).as_expr()

        # Avoid showing '/ 1' for expressions that are not fractions.
        if den_expr == 1:
            return f"({num_expr})"
        else:
            return f"({num_expr}) / ({den_expr})"
    except Exception:
        # Fallback for expressions that are not rational polynomials in s.
        return str(expr)
