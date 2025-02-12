package frc.FSLib2025.math;

public class PolynomialRegression {

    private double[] coefficients;

    public PolynomialRegression (double[] xValues, double[] yValues, int degree) {
        coefficients = fitPolynomial(xValues, yValues, degree);
    }

    public double predictDeg(double xValue) {
        return evaluatePolynomial(coefficients, xValue);
    }

    private double[] fitPolynomial(double[] x, double[] y, int degree) {
        int n = x.length;
        double[][] matrix = new double[degree + 1][degree + 2];
        double[] result = new double[degree + 1];

        for (int i = 0; i <= degree; i++) {
            for (int j = 0; j <= degree; j++) {
                matrix[i][j] = 0;
                for (int k = 0; k < n; k++) {
                    matrix[i][j] += Math.pow(x[k], i + j);
                }
            }
            matrix[i][degree + 1] = 0;
            for (int k = 0; k < n; k++) {
                matrix[i][degree + 1] += Math.pow(x[k], i) * y[k];
            }
        }

        for (int i = 0; i <= degree; i++) {
            for (int k = i + 1; k <= degree; k++) {
                if (Math.abs(matrix[i][i]) < Math.abs(matrix[k][i])) {
                    double[] temp = matrix[i];
                    matrix[i] = matrix[k];
                    matrix[k] = temp;
                }
            }
            for (int k = i + 1; k <= degree; k++) {
                double t = matrix[k][i] / matrix[i][i];
                for (int j = 0; j <= degree + 1; j++) {
                    matrix[k][j] -= t * matrix[i][j];
                }
            }
        }
        for (int i = degree; i >= 0; i--) {
            result[i] = matrix[i][degree + 1] / matrix[i][i];
            for (int k = 0; k < i; k++) {
                matrix[k][degree + 1] -= matrix[k][i] * result[i];
            }
        }

        return result;
    }

    private double evaluatePolynomial(double[] coefficients, double x) {
        double result = 0;
        for (int i = coefficients.length - 1; i >= 0; i--) {
            result = result * x + coefficients[i];
        }
        return result;
    }
}