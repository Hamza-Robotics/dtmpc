from casadi import SX

def generate_casadi_expression_from_string(expression_string):
    # Split the expression string into individual terms
    terms = expression_string.split('+')

    # Initialize an empty set to store the CasADi symbols
    symbols = set()

    # Initialize an empty expression
    expression = 0

    # Parse each term and add it to the expression
    for term in terms:
        # Split each term into factors
        factors = term.split('*')

        # Initialize a term expression
        term_expression = 1

        # Parse each factor and multiply it with the term expression
        for factor in factors:
            factor = factor.strip()  # Remove leading/trailing whitespace
            if factor.startswith('(') and factor.endswith(')'):
                factor = factor[1:-1]  # Remove parentheses if present
            symbol = SX.sym(factor)
            symbols.add(symbol)
            term_expression *= symbol

        # Add the term expression to the overall expression
        expression += term_expression

    # Return the parsed expression and the CasADi symbols
    return expression, symbols

# Example usage
expression_string = "((x_dot - xd_dot)*e_x + (y_dot - yd_dot)*e_y)/e_d"
expression, symbols = generate_casadi_expression_from_string(expression_string)

print("Parsed CasADi expression:")
print(expression)

print("\nCasADi symbols in the expression:")
for symbol in symbols:
    print(symbol)
