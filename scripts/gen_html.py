import re

with open('./doc/ProjectReport.tex', 'r') as f:
  filedata = f.read()

subbed=re.sub(r'.documentclass.*\s|.usepackage.*\s|.begin{document}\s|.maketitle.*\s|.date{.today}\s|.bibliography.*\s|.end{center}.*\s|.begin{center}.*\s|.cite{(.*)}',r'',filedata,re.DOTALL)
subbed=re.sub(r'.title.*\s',r'<html>\n<head></n></head>\n<body></n><h1 align=center>Lab 09 Report</h1><p>',subbed,re.DOTALL)
subbed=re.sub(r'.includegraphics.*{(.*)[.png]*}\s',r'</p><img src = "\1"><p>',subbed,re.DOTALL)
subbed=re.sub(r'.subsection{(.*)}.*\s',r'</p><h4>\1</h4><p>',subbed,re.DOTALL)
subbed=re.sub(r'.section{(.*)}.*\s',r'</p><h2>\1</h2><p>',subbed,re.DOTALL)
subbed=re.sub(r'.end{document}',r'',subbed)

subbed=re.sub(r'(<h1.*</h1>)',r'<font color=#9999FF face="Verdana" size=5>\1</font>',subbed)#h1
subbed=re.sub(r'(<h2.*</h2>)',r'<center><font color=#9999FF face="Verdana" size=4>\1</font></center>',subbed)#h2
subbed=re.sub(r'(<h4.*</h4>)',r'<font color=#66CCFF face="Verdana" size=4>\1</font>',subbed)#h4
subbed=re.sub(r'<p>',r'<font face="Helvetica" size=3><p>',subbed)#p
subbed=re.sub(r'</p>',r'</p></font>',subbed)#p
subbed=re.sub(r'<img(.*)\s',r'<center><img width=400 height=300\1</center>',subbed)#img
subbed=re.sub(r'(Plot .)',r'<b>\1</b>',subbed)

subbed=re.sub(r'(.*)[\\\\][\\\\]\s',r'<center>\1</center>\n',subbed)
subbed=re.sub(r'.texttt{(.*)}}',r'<center>\1</center>\n',subbed)
subbed=re.sub(r'.texttt{(.*)},',r'\1,',subbed)
subbed=re.sub(r'.author{(.*)',r'\1',subbed)
subbed=re.sub(r'(.*).textbf{(.*)}(.*)\s',r'\1<b>\2</b>\3',subbed)

with open('./doc/ProjectReport.html', 'w') as g:
  g.write(subbed)
