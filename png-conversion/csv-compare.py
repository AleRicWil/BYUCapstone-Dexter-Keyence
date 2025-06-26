import csv
import sys

def compare_csvs(file1, file2):
    with open(file1, 'r', newline='', encoding='utf-8') as f1, open(file2, 'r', newline='', encoding='utf-8') as f2:
        reader1 = list(csv.reader(f1))
        reader2 = list(csv.reader(f2))
        
        if len(reader1) != len(reader2):
            print("The CSV files have different numbers of rows.")
            return
        
        for i in range(len(reader1)):
            if len(reader1[i]) != len(reader2[i]):
                print(f"Row {i+1} has different numbers of columns in the two files.")
                return
        
        differences = []
        for row_idx, (row1, row2) in enumerate(zip(reader1, reader2)):
            for col_idx, (cell1, cell2) in enumerate(zip(row1, row2)):
                if abs(float(cell1)-float(cell2)) > .001:
                    differences.append(f"Difference at row {row_idx+1}, column {col_idx+1}: '{cell1}' vs '{cell2}'")
        
        print(f"Comparing {sys.argv[1]}, {sys.argv[2]}")
        if differences:
            print("The CSV files are different")
        else:
            print("The CSV files are identical.")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python csv-compare.py <file1.csv> <file2.csv>")
    else:
        compare_csvs(sys.argv[1], sys.argv[2])

