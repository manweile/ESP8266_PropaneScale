# Documentation Standards
Always use Doxygen Javadoc style documentation for all functions, classes, and methods in C++ and Python.

## General Rules
- Use `/** ... */` for C++ blocks.
- Use triple single quotes `''' ... '''` for Python.
- Start with a `@brief` description.
- Add a `@details` detailed description if necessary.
- Use `@param {type} name description` for all parameters.
- Document all return values.
- Document all exceptions/errors.

## Language Specifics

### C++
- Apply documentation to headers (.h) and implementation files (.cpp, .ino).
- Use `[in]`, `[out]`, or `[in,out]` where applicable, e.g., `@param[in] {int} name description`.
- Use `@return {type} description` for return values.
- Use `@throws {type} description`
- Example:
    /**
    * @brief Calculates the area of a circle.
    * 
    * @details A detailed description if necessary.
    * 
    * @param[in] {double} radius The radius of the circle.
    * @return {double} The calculated area.
    * 
    * @throws {std::invalid_argument} If radius is negative.
    */
double getArea(double radius);

### Python
- Use `@return {type} name description` for all Python return values
- Explicitly include types in the `{type}` format even if type hints are present.
- Use `@exception {type} description`.
- Example:
    def get_area(radius):
        '''
        @brief Calculates the area of a circle.

        @details A detailed description if necessary.

        @param {float} radius The radius of the circle.
        @return {float} area The calculated area.

        @exception ExceptionType Description of exceptions raised.
        '''

        area = 0.00
        area = 3.14 * radius ** 2
        return area
