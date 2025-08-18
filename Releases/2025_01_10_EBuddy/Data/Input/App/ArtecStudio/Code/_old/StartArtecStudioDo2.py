import psutil

# Kill the artec studio app with no saving
def main():
    process_name = "astudio_pro.exe"

    for proc in psutil.process_iter():
        if proc.name() == process_name:
            proc.kill()
    return True

if __name__ == '__main__':
    main()
