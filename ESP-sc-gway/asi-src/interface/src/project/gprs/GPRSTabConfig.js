import React, { Component } from 'react';


import { ENDPOINT_ROOT } from '../../constants/Env';
import SectionContent from '../../components/SectionContent';
import { restComponent } from '../../components/RestComponent';
import LoadingNotification from '../../components/LoadingNotification';

import { TextValidator, ValidatorForm, SelectValidator } from 'react-material-ui-form-validator';

import Button from '@material-ui/core/Button';
import Typography from '@material-ui/core/Typography';
import Slider from '@material-ui/core/Slider';
import { makeStyles } from '@material-ui/core/styles';
import SaveIcon from '@material-ui/icons/Save';

import MenuItem from '@material-ui/core/MenuItem';
import FormControlLabel from '@material-ui/core/FormControlLabel';
import Switch from '@material-ui/core/Switch';

export const GPRS_SETTINGS_ENDPOINT = ENDPOINT_ROOT + "gprsSettings"; // RESTful endpoint

const valueToPercentage = (value) => `${Math.round(value / 255 * 100)}%`;

class GPRSTabConfig extends Component {
  componentDidMount() {
    this.props.loadData();
  }

  render() {
    const { data, fetched, errorMessage, saveData, loadData, handleValueChange, handleSliderChange, handleCheckboxChange} = this.props;
    return (
      <SectionContent title="Configuración de GPRS" titleGutter>
        <LoadingNotification
          onReset={loadData}
          fetched={fetched}
          errorMessage={errorMessage}
          render={() =>
            <GPRSConfigForm
              settings={data}
              onReset={loadData}
              onSubmit={saveData}
              handleSliderChange={handleSliderChange}
              handleValueChange={handleValueChange}
              handleCheckboxChange={handleCheckboxChange}
            />
          }
        />
      </SectionContent>
    )
  }
}

const useStyles = makeStyles(theme => ({
  button: {
    marginRight: theme.spacing(2),
    marginTop: theme.spacing(2),
  },
  selectField: {
    width: "100%",
    marginTop: theme.spacing(2),
    marginBottom: theme.spacing(0.5)
  },
  blinkSpeedLabel: {
    marginBottom: theme.spacing(5),
  },
  switchControl: {
    width: "100%",
    marginTop: theme.spacing(2),
    marginBottom: theme.spacing(0.5)
  },
  textField: {
    width: "100%",
    marginTop: theme.spacing(2),
    marginBottom: theme.spacing(0.5)
  },
}));

function GPRSConfigForm(props) {
  const { settings, onSubmit, onReset, handleValueChange, handleSliderChange, handleCheckboxChange} = props;
  console.log(settings);
  const classes = useStyles();
  return (
    <ValidatorForm onSubmit={onSubmit}>

        <TextValidator
          validators={['matchRegexp:^.{1,100}$']}
          errorMessages={['100 caracteres max']}
          name="apn"
          label="APN"
          className={classes.textField}
          value={settings.apn}
          onChange={handleValueChange('apn')}
          margin="normal"
          />

        <TextValidator
          validators={['matchRegexp:^.{1,100}$']}
          errorMessages={['100 caracteres max']}
          name="user"
          label="User"
          className={classes.textField}
          value={settings.user}
          onChange={handleValueChange('user')}
          margin="normal"
          />

        <TextValidator
          validators={['matchRegexp:^.{1,100}$']}
          errorMessages={['100 caracteres max']}
          name="pass"
          label="Password"
          className={classes.textField}
          value={settings.pass}
          onChange={handleValueChange('pass')}
          margin="normal"
          />
      
      <Button startIcon={<SaveIcon />} variant="contained" color="primary" className={classes.button} type="submit">
        Save
      </Button>
      <Button variant="contained" color="secondary" className={classes.button} onClick={onReset}>
        Reset
      </Button>
    </ValidatorForm>
  );
}

export default restComponent(GPRS_SETTINGS_ENDPOINT, GPRSTabConfig);
