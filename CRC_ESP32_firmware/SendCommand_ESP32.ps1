param (
    [string]$ip,
    [int]$port = 1234,
    [int]$cmd
)

$client = New-Object System.Net.Sockets.TcpClient
$client.Connect($ip, $port)
$stream = $client.GetStream()
$bytes = [byte[]]($cmd)
$stream.Write($bytes, 0, $bytes.Length)
$stream.Close()
$client.Close()