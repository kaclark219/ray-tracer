class Color {
    public:
        int r, g, b;

        // default constructor
        Color() : r(0), g(0), b(0) {}

        // three parameter constructor
        Color(int red, int green, int blue) : r(red), g(green), b(blue) {}

        // use default copy constructor/assignment
        Color(const Color &c) = default;
        Color& operator=(const Color &c) = default;
};