> torrewifi.txt

while true ; do
	
	date +%s >> torrewifi.txt
	sudo iwlist wlp3s0 scanning | egrep 'Address|Signal' >> torrewifi.txt
done

