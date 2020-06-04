def do(a=0 , **options):
    if 'b' in options:
        return a*10
    
    


def main():
    a = 10
    out = do(a=a  , b='Rage')
    return out



if __name__ == "__main__":
    out = main()