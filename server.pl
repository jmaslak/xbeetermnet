#!/usr/bin/perl

#
# Copyright 2014 (C) Joel Maslak
#

use strict;
use warnings;

use autodie;

use local::lib;

use Carp;
use Device::SerialPort;
use Device::XBee::API;
use Data::Dumper;
use Time::HiRes qw/time/;

my $REDISCOVER = 300.0;
my $FORGET_TIME = 600.0;
my $ESCAPETM = 1.0;	# How long after 4 esc's before we give prompt
my $MAX_PENDING_TIME = 3; # How long we wait for an ACK
my $MAXPACKET = 250; 	# Max packet size

my $TERMBRIGHT="\x1b[1m";
my $TERMGREEN="\x1b[32m";
my $TERMYELLOW="\x1b[33m";
my $TERMBLUE="\x1b[34m";
my $TERMCYAN="\x1b[36m";
my $TERMRED="\x1b[31m";

my $TERMCLEAR="\x1bc";
my $TERMNORMAL="\x1b[0m";

my $TERMERR="${TERMBRIGHT}${TERMRED}";
my $CRLF="\r\n";
my $PROMPT="${TERMBRIGHT}${TERMGREEN}SPIDER${TERMYELLOW}->${TERMNORMAL} ";
my $PROMPTLEN=9;

my %ZIGBEE_API_TYPE = (
	136 => 'AT_RESPONSE',
	139 => 'TX_STATUS',
	144 => 'RECEIVE',
	151 => 'REMOTE_AT_RESPONSE',
);
	
my %MAP;
# Example MAP:
#	'13a200:40aebb0a' => '13a200:409f7175',
#	'13a200:409f7175' => '13a200:40aebb0a'

my %DISCOVERY;	# Hash of hashes (keys are "addr" and "type"), key is name
my %HOSTREV;	# Reverse of %DISCOVERY

my %STATE;	# Map of sh:sl to states ("CONNECTED" or "COMMAND")
my %BUFFER;	# Input Buffer
my %OUTBUFFER;	# Output Buffer
my %PENDING;	# Is there pending data to a host?  If so, when was it sent?

my %LASTHEARD;
my %NODEID;
my %FRAMEID;	# Map of sh:sl to node ID (via a hashref)
my %ESCAPE; 	# String of received esc characters
my %ESCAPETM;	# When the last ESC was received

my $LASTDISCOVER; # Time last discovery was performed

my $api;

MAIN: {
	my $serial = Device::SerialPort->new('/dev/ttyUSB1');
	$serial->baudrate(115200);
	$serial->databits(8);
	$serial->stopbits(1);
	$serial->parity('none');
	$serial->handshake('rts');
	$serial->read_char_time(0);
	$serial->read_const_time(1000);

	$api = Device::XBee::API->new(
		{ 
			fh => $serial,
			auto_reuse_frame_id => 1,
			packet_timeout => 1
	       	}
	);

	$LASTDISCOVER = 0;
	while (1) {
		housekeeping();
		receive();

	}
}


sub housekeeping {
	if (scalar(@_) != 0) { confess 'invalid call' }

	foreach my $node (keys %LASTHEARD) {
		if ( ($LASTHEARD{$node} + $FORGET_TIME) < time() ) {
			delete $LASTHEARD{$node};
			delete $NODEID{$node};
			delete $ESCAPE{$node};
			delete $ESCAPETM{$node};
			if (exists($HOSTREV{$node})) {
				delete $DISCOVERY{$HOSTREV{$node}};
				delete $HOSTREV{$node};
			}
			delete $PENDING{$node};
			delete $OUTBUFFER{$node};
		}
	}

	foreach my $node (keys %ESCAPE) {
		if (length($ESCAPE{$node}) == 4) {
			if (($ESCAPETM{$node} + $ESCAPETM) < time()) {
				delete $ESCAPE{$node};
				delete $ESCAPETM{$node};

				command($node, '');
			}
		}
	}

	if ( ($LASTDISCOVER + $REDISCOVER) < time() ) {
		$LASTDISCOVER = time();
		do_discovery();
	}

PEND:	foreach my $node (keys %PENDING) {
		if (!defined($PENDING{$node})) { next PEND; }
		if (($PENDING{$node}->{time} + $MAX_PENDING_TIME) < time()) {
			if ($OUTBUFFER{$node} ne '') {
				free_frame_id($PENDING{$node}->{frame_id});
				$PENDING{$node} = undef;
				# We know outbuffer gets cleared on the below call.
				tx($node, $OUTBUFFER{$node});
			}
		}
	}
}

sub receive {
	if (scalar(@_) != 0) { confess 'invalid call' }

	my $rx = $api->rx();

	if (!defined($rx)) {
		# Nothing to do
		return;
	}

	# Transmission status
	# XXX We should handle this better - we should handle failure
	if (!exists($ZIGBEE_API_TYPE{$rx->{api_type}})) {
		print "UNKNOWN: \n", Dumper($rx), $CRLF;
	} elsif ($ZIGBEE_API_TYPE{$rx->{api_type}} eq 'TX_STATUS') {
		receive_tx_status($rx);
	} elsif ($ZIGBEE_API_TYPE{$rx->{api_type}} eq 'RECEIVE') {
		receive_receive($rx);
	} elsif ($ZIGBEE_API_TYPE{$rx->{api_type}} =~ /^(REMOTE_)?AT_RESPONSE$/) {
		receive_at_response($rx);
	} else {
		if (defined($rx->{frame_id})) {
			free_frame_id($rx->{frame_id});
		}
	}

}

sub receive_tx_status {
	if (scalar(@_) != 1) { confess 'invalid call' }
	my ($rx) = @_;

	if (!exists($FRAMEID{$rx->{frame_id}})) { return; }

	# XXX Convert to using a constant
	if ($rx->{delivery_status} == 50) {
		# This means that the XBee can't handle any traffic
		# from us, probably because it's getting swamped by
		# traffic from other XBees.  So we have to build
		# a massive buffer to handle the backlog.  Otherwise
		# we lose characters, and losing characters is of
		# course a Bad Thing.

		free_frame_id($rx->{frame_id});

		if ($FRAMEID{$rx->{frame_id}}->{type} ne 'TX') { return }

		my $addr = $FRAMEID{$rx->{frame_id}}->{addr};
		my $pkt = $FRAMEID{$rx->{frame_id}}->{data};

		$pkt .= $OUTBUFFER{$addr};
		$OUTBUFFER{$addr} = $pkt;

		# XXX We should play with this a bit.
		$PENDING{$addr}->{time} = (time() - $MAX_PENDING_TIME) + .1;
		print "Delaying...(", length($OUTBUFFER{$addr}), " bytes)\n";
		return;
	}

	# Update NODEID
	my $src = $FRAMEID{$rx->{frame_id}}->{addr};
	$NODEID{$src} = $rx->{remote_na};
		
	update_lastheard($src);

	# Free transmitted frame
	free_frame_id($rx->{frame_id});

	if ($FRAMEID{$rx->{frame_id}}->{type} ne 'TX') { return; }
	if (!defined($PENDING{$src})) { return; }

	if ($PENDING{$src}->{frame_id} == $rx->{frame_id}) {
		$PENDING{$src} = undef;
		if ($OUTBUFFER{$src} ne '') {
			my $data = $OUTBUFFER{$src};
			$OUTBUFFER{$src} = '';
			tx($src, $data);
		}
	}
}

sub receive_receive {
	if (scalar(@_) != 1) { confess 'invalid call' }
	my ($rx) = @_;

	# Get the source address
	my $src = sprintf('%x:%x', $rx->{sh}, $rx->{sl});

	# Set node ID
	$NODEID{$src} = $rx->{na};
		
	update_lastheard($src);

	handle_input($src, $rx->{data});
}

sub receive_at_response {
	if (scalar(@_) != 1) { confess 'invalid call' }
	my ($rx) = @_;

	my $src = sprintf('%x:%x', $rx->{sh}, $rx->{sl});
	my $frameid = $rx->{frame_id};

	# Set node ID
	$NODEID{$src} = $rx->{na};

	update_lastheard($src);

	if (!exists($FRAMEID{$frameid})) { return; }

	if ($rx->{is_ok} and (lc($rx->{command}) eq 'nd')) {
		discover_neighbor($rx);
	}

	if (!($FRAMEID{$frameid}->{type} =~ /^AT( SET)?$/)) {
		return;
	}
	
	free_frame_id($frameid);

	my $addr = $FRAMEID{$frameid}->{addr};

	my $outbuff .= '';

	if ($rx->{is_ok} and (lc($rx->{command}) eq 'nd')) {
		discover_neighbor($rx);
	}
	
	$outbuff .= $CRLF;

	$outbuff .= "${TERMBRIGHT}${TERMCYAN}Received AT Response from ";
	$outbuff .= $src . ' for AT command ' . $rx->{command} . ":$CRLF";
	$outbuff .= "${TERMYELLOW}";

	if (!$rx->{is_ok}) {
		$outbuff .= "  ${TERMERR}Invalid AT command$CRLF";
	} else {
		if ($FRAMEID{$frameid}->{type} eq 'AT SET') {
			$outbuff .= "$TERMCYAN  Value set.$CRLF";
		} else {
			$outbuff .= sprintf("$TERMCYAN    hex $TERMYELLOW\%s$CRLF", hexify($rx->{data}));
			$outbuff .= sprintf("$TERMCYAN  ascii $TERMYELLOW\%s$CRLF", asciify($rx->{data}));
		}
	}
	
	free_frame_id($frameid);

	$outbuff .= $PROMPT;
	if (exists($BUFFER{$addr})) {
		$outbuff .= $BUFFER{$addr};
	};

	tx($addr, $outbuff);
}

sub discover_neighbor {
	if (scalar(@_) != 1) { confess 'invalid call' }
	my ($rx) = @_;

	my $src = sprintf('%x:%x', $rx->{sh}, $rx->{sl});
	my $data = $rx->{data};

	my $info = substr($data, 10);
	$info =~ s/\x00.*$//s;

	my $name;
	my $type;

	my %nodeinfo;
	if ($info =~ /^(TERM|CONSOLE|COORDINATOR|API) [^ ]+$/) {
		($type, $name) = split / /, $info;
	} else {
		$type = 'UNKNOWN';
		$name = $info;
	}

	if ($type eq 'CONSOLE') {
		$STATE{$src} = 'CONSOLE';
	} elsif ($type eq 'COORDINATOR') {
		$STATE{$src} = 'COORDINATOR';
	}

	$name = lc($name);

	if (exists($DISCOVERY{$name})) {
		delete $HOSTREV{$DISCOVERY{$name}->{addr}};
		delete $DISCOVERY{$name};
	}

	if (exists($HOSTREV{$src})) {
		delete $DISCOVERY{$HOSTREV{$src}};
	}

	$nodeinfo{type} = $type;
	$nodeinfo{addr} = $src;
	$DISCOVERY{$name} = \%nodeinfo;
	$HOSTREV{$src} = $name;
}

sub free_frame_id {
	if (scalar(@_) != 1) { confess 'invalid call' }
	my ($frameid) = @_;

	if (exists($FRAMEID{$frameid})) {
		my $src = $FRAMEID{$frameid};

		# Some frames get more than one response, so
		# we don't actually get rid of the $FRAMEID
		# method.  Yes, it's a bit of a race.

		# Free transmitted frame
		$api->free_frame_id($frameid);
	}
}

sub update_lastheard {
	if (scalar(@_) != 1) { confess 'invalid call' }
	my ($src) = @_;

	$LASTHEARD{$src} = time();
}

# Takes binary data and displays only printable characters
sub asciify {
	if (scalar(@_) != 1) { confess 'invalid call' }
	my ($data) = @_;

	$data =~ s/[^\x20-\x7e]/./gs;
	return $data;
}

# Takes a hex string and turns it into the binary data that
# it represents.
sub binify {
	if (scalar(@_) != 1) { confess 'invalid call' }
	my ($data) = @_;

	my $out = '';
	my (@chars) = split //, $data;
	while (scalar(@chars)) {
		my $a = shift @chars;
		my $b = shift @chars;

		$out .= chr(hex($a . $b));
	}
	
	return $out;
}

# Takes binary data and turns it into a hex string
sub hexify {
	if (scalar(@_) != 1) { confess 'invalid call' }
	my ($data) = @_;

	my (@chars) = split //, $data;

	my $out = '';
	foreach my $char (@chars) {
		$out .= sprintf('%02x', ord($char));
	}

	return $out;
}

sub handle_input {
	if (scalar(@_) != 2) { confess 'invalid call' }
	my ($src, $data) = @_;

	if (exists($STATE{$src}) and ($STATE{$src} eq 'COMMAND')) {
		command($src, $data);
	} elsif (!exists($MAP{$src})) {
		command($src, $data);
	} else {
		my $addr = $MAP{$src};

		if (($data =~ /\x1b+$/) and
		    ((!exists($HOSTREV{$src})) or
		     ($DISCOVERY{$HOSTREV{$src}}->{type} ne 'CONSOLE'))) {

			my $esc = $data;
			$esc =~ s/^[^\x1b]+//;

			# ESC must be 1 sec duration max between escapes
			if (exists($ESCAPETM{$src}) and ($ESCAPETM{$src}+1 < time())) {
				tx($addr, $ESCAPE{$src});

				delete $ESCAPE{$src};
				delete $ESCAPETM{$src};
			}

			if (length($esc) ne length($data)) {
				$ESCAPE{$src} = $esc;
			} else {
				$ESCAPE{$src} .= $esc;
			}
			$ESCAPETM{$src} = time();
		} else {
			if (exists($ESCAPE{$src})) { tx($addr, $ESCAPE{$src}); }

			delete $ESCAPE{$src};
			delete $ESCAPETM{$src};
		
			tx($addr, $data);
		}
	}
}

sub command {
	if ((scalar(@_) < 2) or (scalar(@_) > 3)) { confess 'invalid call' }
	my ($addr, $data, $err) = @_;

	# Ignore packets in command mode from hosts
	if (exists($HOSTREV{$addr}) and ($DISCOVERY{$HOSTREV{$addr}}->{type} eq 'CONSOLE')) {
		if (exists($STATE{$addr}) and ($STATE{$addr} eq 'COMMAND')) {
			delete $STATE{$addr};
		}
		return;
	}

	if ((!exists($STATE{$addr})) or ($STATE{$addr} ne 'COMMAND')) {
		$STATE{$addr} = 'COMMAND';
		$BUFFER{$addr} = ''; # Throw away data.

		delete $ESCAPE{$addr};
		delete $ESCAPETM{$addr};

		# Let people get out of a problem
		$OUTBUFFER{$addr} = '';

		if (exists($MAP{$addr})) {
			if (exists($HOSTREV{$MAP{$addr}})) {
				my $dhost = $HOSTREV{$MAP{$addr}};
				if ($DISCOVERY{$dhost}->{type} eq 'TERM') {
					command($dhost, '');
				}
			}

			delete $MAP{$MAP{$addr}};
			delete $MAP{$addr};
		}

		if (defined($err)) {
			tx($addr, $TERMCLEAR . $err);
			send_prompt($addr, undef);
		} else {
			send_prompt($addr, 1);
		}

		return;
	}

	my $outbuff = '';

	for my $char (split(//, $data)) {
		if ( (ord($char) == 8) or (ord($char) == 127) ) {
			# Handle backspace
			if (length($BUFFER{$addr}) > 0) {
				$outbuff .= "\x08 \x08";
				$BUFFER{$addr} = substr($BUFFER{$addr}, 0, -1);
			}
		} elsif (ord($char) == 13) {

			# Do processing.
			my $params = [];
			my $cmd;
			($cmd, @$params) = split / /, $BUFFER{$addr};
			if (!defined($cmd)) { $cmd = ''; }
			$cmd = lc($cmd);

			my $paramcnt = scalar(@$params);

			$outbuff .= $CRLF;
	
			if (($cmd eq 'activity') and ($paramcnt == 0)) {
				$outbuff .= cmd_activity($addr, $cmd, $params);
			} elsif (($cmd eq 'at') and ($paramcnt == 2)) {
				$outbuff .= cmd_atq($addr, $cmd, $params);
			} elsif (($cmd eq 'at?') and ($paramcnt == 2)) {
				$outbuff .= cmd_atq($addr, $cmd, $params);
			} elsif (($cmd eq 'atx') and ($paramcnt == 3)) {
				$outbuff .= cmd_atx($addr, $cmd, $params);
			} elsif (($cmd eq 'ata') and ($paramcnt >= 3)) {
				$outbuff .= cmd_ata($addr, $cmd, $params);
			} elsif (($cmd eq 'clear') and ($paramcnt == 0)) {
				$outbuff .= cmd_clear($addr, $cmd, $params);
			} elsif ((($cmd eq 'connect') or ($cmd eq 'c')) and ($paramcnt == 1)) {
				$outbuff .= cmd_connect($addr, $cmd, $params);
			} elsif (($cmd eq 'disconnect') and ($paramcnt == 1)) {
				$outbuff .= cmd_disconnect($addr, $cmd, $params);
			} elsif (($cmd eq 'discover') and ($paramcnt == 0)) {
				$outbuff .= cmd_discover($addr, $cmd, $params);
			} elsif ((($cmd eq 'help') or ($cmd eq '?')) and ($paramcnt == 0)) {
				$outbuff .= cmd_help();
			} elsif ((($cmd eq 'hosts') or ($cmd eq '?')) and ($paramcnt == 0)) {
				$outbuff .= cmd_hosts();
			} elsif (($cmd eq 'takeover') and ($paramcnt == 1)) {
				$outbuff .= cmd_takeover($addr, $cmd, $params);
			} elsif (($paramcnt == 0) and ($cmd eq 'whoami')) {
				$outbuff .= cmd_whoami($addr, $cmd, $params);
			} else {
				if ($cmd ne '') {
					$outbuff .= "${TERMERR}Invalid command (use HELP for list of valid commands)$CRLF";
				}
				$outbuff .= $PROMPT;
			}

			$BUFFER{$addr} = '';
		
		} elsif ($char =~ /[A-Za-z0-9\?: ]/) {
			# Alphanumerics and other limited characters
			$BUFFER{$addr} .= $char;
			$outbuff .= $char;
		} else {
			# Invalid character
			$outbuff .= "\x07";
		}
	}

	if (length($outbuff) > 0) {
		tx($addr, $outbuff);
	}
}

sub send_prompt {
	if (scalar(@_) != 2) { confess 'invalid call' }
	my ($addr, $clear) = @_;

	if ($clear) {
		tx($addr, "${TERMCLEAR}${PROMPT}");
	} else {
		tx($addr, "${PROMPT}");
	}
}

sub tx {
	if (scalar(@_) != 2) { confess 'invalid call' }
	my ($addr, $data) = @_;

	if (!defined($data)) { confess 'BAD DATA - ASSERT' };

	my $dest = get_destination($addr);

	if (!exists($OUTBUFFER{$addr})) { $OUTBUFFER{$addr} = ''; }
	if (!exists($PENDING{$addr}))   { $PENDING{$addr} = undef; }

	# $PENDING{$addr} = undef;
	if ($PENDING{$addr}) {
		$OUTBUFFER{$addr} .= $data;
		return;
	}

	my $pkt = substr($data, 0, $MAXPACKET);
	if (length($data) <= $MAXPACKET) {
		$data = '';
	} else {
		# NOTE WHAT WE DO HERE!
		$data = substr($data, $MAXPACKET);
		$OUTBUFFER{$addr} = $data;
	}

	my $frameid = $api->tx($dest, $pkt, 1);
	$FRAMEID{$frameid} = {
		addr => $addr,
		type => 'TX',
		data => $pkt
	};

	$PENDING{$addr}->{time} = time();
	$PENDING{$addr}->{frame_id} = $frameid;
}

sub get_destination {
	if (scalar(@_) != 1) { confess 'invalid call' }
	my ($addr) = @_;

	$addr = lc($addr);

	if (!($addr =~ /^([0-9a-f]{1,8}):([0-9a-f]{1,8})$/)) {
		if (exists($DISCOVERY{$addr})) {
			$addr = $DISCOVERY{$addr}->{addr};
		} else {
			# Not a valid address
			return undef;
		}
	}

	# We know we have a valid address now.

	my ($h, $l) = split /:/, $addr;

	my $dest = {
		sh => hex $h,
		sl => hex $l,
		na => $NODEID{$addr}
	};

	return $dest;
}

sub get_display_host {
	if (scalar(@_) != 1) { confess 'invalid call' }
	my $addr = shift;

	if (exists($HOSTREV{$addr})) {
		$addr .= ' (' . $HOSTREV{$addr} . ')'
	}

	return $addr;
}

sub cmd_help {
	if (scalar(@_) != 0) { confess 'invalid call' }

	my $outbuff = '';

	my $Y = $TERMYELLOW;
	my $C = $TERMCYAN;

	$outbuff .= "${TERMBRIGHT}$CRLF";
	$outbuff .= "${C}Commands:$CRLF";
	$outbuff .= "$Y ACTIVITY                    $C Show all managed connections$CRLF";
	$outbuff .= "$Y AT? <Addr> <AT CMD>         $C Query for AT command$CRLF";
	$outbuff .= "$Y ATA <Addr> <AT CMD> <STR>   $C Set AT value using binary string$CRLF";
	$outbuff .= "$Y ATX <Addr> <AT CMD> <HEX>   $C Set AT value using hex string$CRLF";
	$outbuff .= "$Y CLEAR                       $C Clear screen$CRLF";
	$outbuff .= "$Y CONNECT <hostname>          $C Connect to a host$CRLF";
	$outbuff .= "$Y DISCONNECT <hostname>       $C Close open connections to host$CRLF";
	$outbuff .= "$Y DISCOVER                    $C Perform network discovery$CRLF";
	$outbuff .= "$Y HELP                        $C This screen$CRLF";
	$outbuff .= "$Y HOSTS                       $C Display all discovered hosts$CRLF";
	$outbuff .= "$Y TAKEOVER <hostname>         $C Takeover connection to host$CRLF";
	$outbuff .= "$Y WHOAMI                      $C Return your network address$CRLF";
	
	$outbuff .= $CRLF;
	$outbuff .= $PROMPT;

	return $outbuff;
}

sub cmd_hosts {
	if (scalar(@_) != 0) { confess 'invalid call' }

	my $outbuff = '';

	my $Y = $TERMYELLOW;
	my $C = $TERMCYAN;

	$outbuff .= "${TERMBRIGHT}$CRLF";
	$outbuff .= "${C}Available hosts:$CRLF";
	$outbuff .= $Y;

	foreach my $host (sort keys(%DISCOVERY)) {
		$outbuff .= sprintf("  %-20s %-17s (%s)$CRLF", $host, $DISCOVERY{$host}->{addr},
				    $DISCOVERY{$host}->{type});
	}

	$outbuff .= $CRLF;
	$outbuff .= $PROMPT;

	return $outbuff;
}

sub cmd_connect {
	if (scalar(@_) != 3) { confess 'invalid call' }
	my ($addr, $cmd, $params) = @_;

	my $outbuff = '';

	my $dhost = lc($params->[0]);

	# XXX We should allow connection to unknown hosts
	if (!exists($DISCOVERY{$dhost})) {
		$outbuff .= "${TERMERR}Invalid host name (use HELP to get host list)$CRLF";
		$outbuff .= $PROMPT;

		return $outbuff;
	}

	my $addr2 = $DISCOVERY{$dhost}->{addr};

	if ($addr eq $addr2) {
		$outbuff .= "${TERMERR}Cannot connect to self$CRLF";
		$outbuff .= $PROMPT;

		return $outbuff;
	}

	if ((exists($MAP{$addr2})) and ($MAP{$addr2} ne $addr)) {
		$outbuff .= "${TERMERR}Host busy$CRLF";
		$outbuff .= $PROMPT;

		return $outbuff;
	}

	my $src = get_display_host($addr);
	my $dst = get_display_host($addr2);
		
	$outbuff .= "${TERMBRIGHT}${TERMGREEN}Connecting...";
	$outbuff .= "$src to $dst...$CRLF";
	$outbuff .= "${TERMYELLOW}Hit ESC four times to return to Spider.$CRLF$TERMNORMAL";
	$outbuff .= $CRLF;

	if ($DISCOVERY{$dhost}->{type} eq 'TERM') {
		my $outbuff2 = "${CRLF}${TERMBRIGHT}${TERMGREEN}Connecting by remote user request...$CRLF";
		$outbuff2 .= "$dst to $src...$CRLF";
		$outbuff2 .= "${TERMYELLOW}Hit ESC four times to return to Spider.$CRLF$TERMNORMAL";
		$outbuff2 .= $CRLF;
		tx($addr2, $outbuff2);
	}

	if (exists($MAP{$addr})) {
		delete $MAP{$MAP{$addr}};
	}

	$MAP{$addr} = $addr2;
	$MAP{$addr2} = $addr;

	$STATE{$addr} = 'CONNECTED';

	# Send newline to host
	tx($addr2, "\n");

	return $outbuff;
}

sub cmd_whoami {
	if (scalar(@_) != 3) { confess 'invalid call' }
	my ($addr, $cmd, $params) = @_;

	my $outbuff = '';

	my $me = get_display_host($addr);

	$outbuff .= "${TERMBRIGHT}${TERMCYAN}You are ${TERMYELLOW}$me$CRLF";
	$outbuff .= $PROMPT;

	return $outbuff;
}

sub cmd_activity {
	if (scalar(@_) != 3) { confess 'invalid call' }
	my ($addr, $cmd, $params) = @_;

	my $outbuff = '';

	$outbuff .= "${TERMBRIGHT}${TERMCYAN}Current activity:$CRLF${TERMYELLOW}";

NEXTACTIVE:
	foreach my $host (keys %DISCOVERY) {
		my $addr2 = $DISCOVERY{$host}->{addr};
		my $dst = get_display_host($addr2);

		if (!exists($MAP{$addr2})) {
			if (exists($STATE{$addr2})) {
				$outbuff .= "   $dst -- ";
				$outbuff .= lc($STATE{$addr2});
				$outbuff .= $CRLF;
			} else {
				$outbuff .= "   $dst -- idle$CRLF";
			}
			next NEXTACTIVE;
		}

		my $src = get_display_host($MAP{$addr2});
		$outbuff .= "   $src to $dst$CRLF";
	}

	foreach my $addr2 (keys %STATE) {
		if ($STATE{$addr2} ne 'CONNECTED') {
			if (!exists($HOSTREV{$addr2})) {
				my $src = get_display_host($addr2);

				if ($addr2 eq $addr) {
					$outbuff .= ' * ';
				} else {
					$outbuff .= '   ';
				}

				$outbuff .= "$src -- " . lc($STATE{$addr2}) . $CRLF;
			}
		}
	}

	$outbuff .= ${PROMPT};

	return $outbuff;
}

sub cmd_disconnect {
	if (scalar(@_) != 3) { confess 'invalid call' }
	my ($addr, $cmd, $params) = @_;

	my $outbuff = '';

	my $dhost = lc($params->[0]);

	# XXX Can't target users yet
	if ((!exists($DISCOVERY{$dhost})) or ($DISCOVERY{$dhost}->{type} ne 'CONSOLE')) {
		$outbuff .= "${TERMERR}Invalid host name (use HELP to get host list)$CRLF";
		$outbuff .= $PROMPT;

		return $outbuff;
	}

	if (!exists($MAP{$DISCOVERY{$dhost}->{addr}})) {
		$outbuff .= "${TERMERR}Host not in use$CRLF";
		$outbuff .= $PROMPT;

		return $outbuff;
	}

	my $remote = $MAP{$DISCOVERY{$dhost}->{addr}};

	# Kick other user off.
	delete $MAP{$remote};
	delete $MAP{$DISCOVERY{$dhost}->{addr}};

	# Tell other user we kicked them off.
	my $err = "${TERMERR}You were disconnected from $dhost by $addr$CRLF";
	command($remote, '', $err);

	$outbuff .= "${TERMBRIGHT}${TERMCYAN}Disconnected $remote from $dhost$CRLF";

	if ($cmd eq 'disconnect') { $outbuff .= $PROMPT; }

	return $outbuff;
}

sub cmd_takeover {
	if (scalar(@_) != 3) { confess 'invalid call' }
	my ($addr, $cmd, $params) = @_;

	my $outbuff = '';

	my $dhost = lc($params->[0]);

	if (!exists($DISCOVERY{$dhost})) {
		$outbuff .= "${TERMERR}Invalid host name (use HELP to get host list)$CRLF";
		$outbuff .= $PROMPT;

		return $outbuff;
	}

	if (!exists($MAP{$DISCOVERY{$dhost}->{addr}})) {
		$outbuff .= "${TERMERR}Host not in use$CRLF";
		$outbuff .= $PROMPT;

		return $outbuff;
	}

	my $disconnect = cmd_disconnect($addr, $cmd, $params);
	my $connect = cmd_connect($addr, $cmd, $params);
	my $foo = 1; # Bug in Perl!!!!  This is the workaround.  Without this,
	             # it seems that $outbuff gets wiped (!).

	$outbuff .= $disconnect . $connect;

	return $outbuff;
}

sub cmd_atq {
	if (scalar(@_) != 3) { confess 'invalid call' }
	my ($addr, $cmd, $params) = @_;

	my $outbuff = '';

	my $dhost = get_destination($params->[0]);
	my $atcmd = $params->[1];

	if (!defined($dhost)) {
		$outbuff .= "${TERMERR}Invalid host name (use HELP to get host list)$CRLF";
		$outbuff .= $PROMPT;
		return $outbuff;
	}

	if (!($atcmd =~ /^..$/)) {
		$outbuff .= "${TERMERR}Invalid AT command (should be two characters)$CRLF";
		$outbuff .= $PROMPT;
		return $outbuff;
	}

	$dhost->{apply_changes} = 1;
	my $frameid = $api->remote_at($dhost, $atcmd);

	$FRAMEID{$frameid} = {
		addr => $addr,
		type => 'AT'
	};

	$outbuff .= $PROMPT;
	return $outbuff;
}

sub cmd_atx {
	if (scalar(@_) != 3) { confess 'invalid call' }
	my ($addr, $cmd, $params) = @_;

	my $outbuff = '';

	my $dhost = get_destination($params->[0]);
	my $atcmd = $params->[1];
	my $athex = lc($params->[2]);

	if (!defined($dhost)) {
		$outbuff .= "${TERMERR}Invalid host name (use HELP to get host list)$CRLF";
		$outbuff .= $PROMPT;
		return $outbuff;
	}

	if (!($atcmd =~ /^..$/)) {
		$outbuff .= "${TERMERR}Invalid AT command (should be two characters)$CRLF";
		$outbuff .= $PROMPT;
		return $outbuff;
	}

	if (!($athex =~ /^([0-9a-f]{2})+$/)) {
		$outbuff .= "${TERMERR}Invalid AT command parameters (should be hex string)$CRLF";
		$outbuff .= $PROMPT;
		return $outbuff;
	}

	$dhost->{apply_changes} = 1;
	my $frameid = $api->remote_at($dhost, $atcmd, binify($athex));

	$FRAMEID{$frameid} = {
		addr => $addr,
		type => 'AT SET'
	};

	$outbuff .= $PROMPT;
	return $outbuff;
}

sub cmd_ata {
	if (scalar(@_) != 3) { confess 'invalid call' }
	my ($addr, $cmd, $params) = @_;

	my $dhost = shift @$params;
	my $atcmd = shift @$params;
	my $str = join ' ', @$params;

	return cmd_atx($addr, $cmd, [ $dhost, $atcmd, hexify($str) ]);
}

sub cmd_discover {
	if (scalar(@_) != 3) { confess 'invalid call' }
	my ($addr, $cmd, $params) = @_;

	my $outbuff = "${TERMBRIGHT}${TERMCYAN}Discovering network...$CRLF";
	$outbuff .= $PROMPT;

	do_discovery();

	return $outbuff;
}

sub cmd_clear {
	if (scalar(@_) != 3) { confess 'invalid call' }
	my ($addr, $cmd, $params) = @_;

	my $outbuff = $TERMCLEAR;
	$outbuff .= $PROMPT;

	return $outbuff;
}

sub do_discovery {
	if (scalar(@_) != 0) { confess 'invalid call' }

	my $frameid = $api->at('nd', '');
	$FRAMEID{$frameid} = {
		type => 'DISCOVERY'
	}
}


